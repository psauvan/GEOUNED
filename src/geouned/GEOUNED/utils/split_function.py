import BOPTools.SplitAPI


def split_bop(solid, tools, tolerance, options, scale=0.1):

    if tolerance >= 0.1:
        compSolid = BOPTools.SplitAPI.slice(solid, tools, "Split", tolerance=tolerance)

    elif tolerance < 1e-12:
        if options.scaleUp:
            tol = 1e-13 if options.splitTolerance == 0 else options.splitTolerance
            compSolid = split_bop(solid, tools, tol / scale, options, 1.0 / scale)
        else:
            compSolid = BOPTools.SplitAPI.slice(solid, tools, "Split", tolerance=tolerance)

    else:
        try:
            compSolid = BOPTools.SplitAPI.slice(solid, tools, "Split", tolerance=tolerance)
        except:
            compSolid = split_bop(solid, tools, tolerance * scale, options, scale)

    return compSolid
