import math

twoPi = 2 * math.pi
halfPi = 0.5 * math.pi
threehalfPi = 1.5 * math.pi

# Absolute floor (mm) for a *relative* surface-matching tolerance
# (`Tolerances.relativeTol=True`). Those tolerances are `rel * |position|`,
# which is exactly 0 for a surface sitting at the origin -- and a
# tolerance of 0 rejects even two bit-identical surfaces (`|0| < 0` is
# false), so e.g. every plane z=0 would get its own surface card. 1e-9 mm
# is far above float noise on any realistic coordinate (~2e-11 mm at 1e5 mm)
# and far below any real geometric difference.
RELATIVE_TOL_ABS_FLOOR = 1.0e-9


class mask:
    fwd_cyl = 1  # orientation mask (False/True)
    p1_cyl = 2  # P1/Cyl configuration   (OR/AND)
    p2_cyl = 4  # P2/Cyl configuration   (OR/AND)
    p1_p2 = 8  # add bracket to or operator : True : n1(nd:n2) , False: n1 nd : n2
    p1_pd = 16  # pc/P1 configuration    (OR/AND)
    p2_pd = 32  # pc/P2 configuration    (OR/AND)
    same_p1_pd = 64
    same_p2_pd = 128
