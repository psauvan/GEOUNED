import math

twoPi = 2 * math.pi
halfPi = 0.5 * math.pi
threehalfPi = 1.5 * math.pi


class mask:
    # (X/Y) X=0 Y=1
    fwd_cyl = 1  # orientation mask (False/True)
    p1_cyl = 2  # P1/Cyl configuration   (OR/AND)
    p2_cyl = 4  # P2/Cyl configuration   (OR/AND)
    p1_p2 = 8  # P1/P2 configuration     (OR/AND)
    # p1_pc = 16 # pc/P1 configuration    (OR/AND)
    # p2_pc = 32 # pc/P2 configuration    (OR/AND)
    pc_side = 16  # p1 p2 fold to the same cylinde side (False/True)
    cross_in = 32  # p1 p2 cross inside cylinder (False/True)
    inter_v1 = 64  # p1 p2 intersection point toward p1 direction (False/True)
    fwd_corner = 128  # round corner orientation (False/True)

    fwd_cyl = 1  # orientation mask (False/True)
    p1_cyl = 2  # P1/Cyl configuration   (OR/AND)
    p2_cyl = 4  # P2/Cyl configuration   (OR/AND)
    p1_p2 = 8  # add bracket to or operator : True : n1(nd:n2) , False: n1 nd : n2
    p1_pd = 16  # pc/P1 configuration    (OR/AND)
    p2_pd = 32  # pc/P2 configuration    (OR/AND)
    notp1 = 64  # oposite plane normal vector
    notp2 = 128  # oposite plane normal vector
    same_p1_pd = 256
    same_p2_pd = 512
