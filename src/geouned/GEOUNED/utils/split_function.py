import BOPTools.SplitAPI
import Part


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


def single_tool_split(solid_in, tools, tolerance, options, scale=0.1):

    solid_list = [solid_in]
    for t in tools:
        new_list = []
        for solid in solid_list:
            cut = split_bop(solid, (t,), tolerance, options, scale)
            if len(cut.Solids) == 0:
                new_list.append(solid)
            else:
                new_list.extend(cut.Solids)
        solid_list = new_list
    return Part.makeCompound(solid_list)
