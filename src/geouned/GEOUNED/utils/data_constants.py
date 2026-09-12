import math

twoPi = 2 * math.pi
halfPi = 0.5 * math.pi
threehalfPi = 1.5 * math.pi


class mask:
    fwd_cyl = 1  # orientation mask (False/True)
    p1_cyl = 2  # P1/Cyl configuration   (OR/AND)
    p2_cyl = 4  # P2/Cyl configuration   (OR/AND)
    p1_p2 = 8  # add bracket to or operator : True : n1(nd:n2) , False: n1 nd : n2
    p1_pd = 16  # pc/P1 configuration    (OR/AND)
    p2_pd = 32  # pc/P2 configuration    (OR/AND)
    same_p1_pd = 64
    same_p2_pd = 128
