from ....geo import GSolid, Gfuse, Gmake_compound, Gsplit


class SplitBase:
    def __init__(self, base, knownSurf={}, orientation="Forward"):
        self.base = base
        self.knownSurf = knownSurf
        self.orientation = orientation


def joinBase(baseList):
    shape = []
    surf = {}
    removedKeys = []
    fwd = True
    for b in baseList:
        if b.orientation == "Reversed":
            fwd = False
        if b.base is not None:
            shape.append(b.base)
        for k, v in b.knownSurf.items():
            if k in removedKeys:
                continue
            if k not in surf.keys():
                surf[k] = v
            else:
                if surf[k] == v:
                    continue
                else:
                    surf[k] = None
                    removedKeys.append(k)

    newbase = FuseSolid(shape)
    orientation = "Forward" if fwd else "Reversed"
    return SplitBase(newbase, surf, orientation)


# TODO rename this function as there are two with the name name
def SplitSolid(base, surfacesCut, cellObj, tolerance=0.01):  # 1e-2
    # split Base (shape Object or list/tuple of shapes)
    # with selected surfaces (list of surfaces objects) cutting the base(s) (surfacesCut)
    # cellObj is the CAD object of the working cell to reconstruction.
    # the function return a list of solids enclosed fully in the cell (fullPart)
    # and a list of solids not fully enclosed in the cell (cutPart). These lasts
    # will require more splitting with the others surfaces defining the cell.

    fullPart = []
    cutPart = []

    # part if several base in input

    if type(base) is list or type(base) is tuple:
        for b in base:
            fullList, cutList = SplitSolid(b, surfacesCut, cellObj, tolerance=tolerance)
            fullPart.extend(fullList)
            cutPart.extend(cutList)
        return fullPart, cutPart

    # part if base is shape object
    # resulting cell orientation is "Reversed" only if both
    # cells have reversed orientations
    orientation = "Forward"

    if abs(base.base.Volume / base.base.Area) < 1e-2:
        return fullPart, cutPart

    Tools = tuple(s.shape for s in surfacesCut)
    if Tools[0] is not None:
        result = Gsplit(GSolid(base.base), GSolid(Tools[0]), tolerance)
        Solids = [s.__native__ for s in result.solids]
    else:
        Solids = [base.base]

    partPositions, partSolids = space_decomposition(Solids, surfacesCut)

    for pos, sol in zip(partPositions, partSolids):
        # fullPos = updateSurfacesValues(pos,cellObj.surfaces,base.knownSurf)
        # inSolid = cellObj.definition.evaluate(fullPos)

        pos.update(base.knownSurf)
        inSolid = cellObj.definition.evaluate(pos)
        inSolid = inSolid if type(inSolid) is bool else None
        # if solidTool :
        #  ii += 1
        #  print(solidTool)
        #  print(cellObj.definition)
        #  print(pos)
        #  print('eval',inSolid)
        #  name = str(cellObj.definition)
        #  sol.exportStep('solid_{}{}.stp'.format(name,ii))

        if inSolid:
            fullPart.append(SplitBase(sol, pos, orientation))
        elif inSolid is None:
            cutPart.append(SplitBase(sol, pos, orientation))
    return fullPart, cutPart


# Get the position of subregion with respect
# all cutting surfaces
def space_decomposition(solids, surfaces):

    component = []
    good_solids = []
    for c in solids:
        if c.Volume < 1e-3:
            if abs(c.Volume) < 1e-3:
                continue
            else:
                c.reverse()
                print("Negative solid Volume", c.Volume)
        Svalues = {}
        point = GSolid(c).find_interior_point()
        if point is None:
            continue  # point not found in solid (solid is surface or very thin can be source of lost particules in MCNP)
        for surf in surfaces:
            Svalues[surf.id] = surface_side(point, surf)

        component.append(Svalues)
        good_solids.append(c)
    return component, good_solids


# check the position of the point with respect to a surface
#
# Delegates straight to the wrapped geo descriptor's own is_inside()
# (GVector math) instead of reimplementing the same plane/cylinder/cone/
# sphere formulas natively a second time -- see CellSurface in Objects.py
# and boolean_solids.check_sign_primitive, which has the identical
# formulas in the (separate) main-decomposition point-classification path.
#
# The old version here also handled several surface types (cone_elliptic,
# hyperboloid, ellipsoid, cylinder_elliptic, cylinder_hyperbolic,
# paraboloid, generic torus, box) that build_region.get_surface() never
# actually constructs -- confirmed unreachable in this pipeline (inherited
# from a broader original toolkit) and dropped along with the now-unused
# btwPPlanes() helper (truncated-cylinder/cone support) they depended on.
def surface_side(p, surf):
    return surf.is_inside(p)


# ************************************************


def FuseSolid(parts):
    if (len(parts)) <= 1:
        if parts:
            solid = parts[0]
        else:
            return None
    else:
        gparts = [GSolid(p) for p in parts]
        try:
            fused = Gfuse(gparts)
        except Exception:
            fused = None

        if fused is not None:
            try:
                refined = fused.refine()
            except Exception:
                refined = fused

            if refined.is_valid():
                gsolid = refined
            elif fused.is_valid():
                gsolid = fused
            else:
                gsolid = Gmake_compound(gparts)
        else:
            gsolid = Gmake_compound(gparts)
        solid = gsolid.__native__

    if solid.Volume < 0:
        solid = GSolid(solid).reverse().__native__
    return solid
