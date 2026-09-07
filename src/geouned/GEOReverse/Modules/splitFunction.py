import math

from ...geo import GSolid, Gfuse_solids, Gsplit


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

    newbase = Gfuse_solids(shape)
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
    if cellObj.boundBox.Orientation == base.orientation:
        orientation = cellObj.boundBox.Orientation
    else:
        orientation = "Forward"

    if abs(base.base.Volume / base.base.Area) < 1e-2:
        return fullPart, cutPart

    # SplitSolid is always called with exactly one cutting surface
    # (`(p,)`/`(surf,)` at every call site in buildSolidCell.py) -- the
    # tuple form is legacy, only its first (only) element is ever used.
    tool = surfacesCut[0].shape
    if tool is not None:
        try:
            Solids = [s.__native__ for s in Gsplit(base.base, tool, tolerance=tolerance).solids]
        except Exception:
            Solids = []
        if not Solids:
            Solids = [base.base.__native__]
        Solids = [GSolid(s) for s in Solids]
    else:
        Solids = [base.base]

    partPositions, partSolids = space_decomposition(Solids, surfacesCut)

    for pos, sol in zip(partPositions, partSolids):
        # fullPos = updateSurfacesValues(pos,cellObj.surfaces,base.knownSurf)
        # inSolid = cellObj.definition.evaluate(fullPos)

        pos.update(base.knownSurf)
        inSolid = cellObj.definition.evaluate(pos)

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


def updateSurfacesValues(position, surfaces, knownSurf):
    position.update(knownSurf)
    sname = set(surfaces.keys())
    pname = set(position.keys())

    fullpos = position.copy()
    for name in sname.difference(pname):
        fullpos[name] = None
    return fullpos


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
                c = c.reverse()
                print("Negative solid Volume", c.Volume)
        Svalues = {}
        point = c.find_interior_point()
        if point is None:
            continue  # point not found in solid (solid is surface or very thin can be source of lost particules in MCNP)
        for surf in surfaces:
            Svalues[surf.id] = surface_side(point, surf)

        component.append(Svalues)
        good_solids.append(c)
    return component, good_solids


# check the position of the point with respect
# a surface
def surface_side(p, surf):
    if surf.type == "sphere":
        org, R = surf.params
        D = p - org
        inout = D.length - R

    elif surf.type == "plane":
        normal, d = surf.params
        inout = p.dot(normal) - d

    elif surf.type == "cylinder":
        P, v, R = surf.params

        D = p - P
        if not surf.truncated:
            inout = D.cross(v).length - R
        else:
            inCyl = D.cross(v).length / v.length - R  # <0 in cylinder
            inPln = btwPPlanes(p, P, v)  # <0  between planes

            if (inCyl < 0) and (inPln < 0):
                inout = -1  # inside the can
            else:
                inout = 1  # outside the can

    elif surf.type == "cone":
        if not surf.truncated:
            P, v, t, dblsht = surf.params
            X = (p - P).normalized()
            dprod = X.dot(v)
            dprod = max(-1, min(1, dprod))
            a = math.acos(dprod) if not dblsht else math.acos(abs(dprod))
            inout = a - math.atan(t)
        else:
            P, v, R1, R2 = surf.params
            apex = P + R1 / (R1 - R2) * v

            X = (p - apex).normalized()
            dprod = X.dot(-v) / v.length  # -v because reverse axis. in MCNP TRC r1 > r2
            dprod = max(-1, min(1, dprod))
            a = math.acos(dprod)

            t = (R1 - R2) / v.length
            inCone = a - math.atan(t)
            inPln = btwPPlanes(p, P, v)  # <0  between planes

            if (inCone < 0) and (inPln < 0):
                inout = -1  # inside the can
            else:
                inout = 1  # outside the can

    elif surf.type == "cone_elliptic":
        apex, axis, Ra, radii, rAxes, dblsht = surf.params

        r = p - apex
        X = r.dot(rAxes[1])
        Y = r.dot(rAxes[0])
        Z = r.dot(axis)
        if dblsht:
            Z = abs(Z)
        inout = (X / radii[1]) ** 2 + (Y / radii[0]) ** 2 - Z / Ra

    elif surf.type == "hyperboloid":
        center, axis, radii, rAxes, onesht = surf.params

        r = p - center
        rX = r.dot(rAxes[1])
        v = r - (rX * rAxes[1] + center)
        d = v.length

        one = 1 if onesht else -1
        radical = (rX / radii[1]) ** 2 + one

        if radical > 0:
            Y = radii[0] * math.sqrt(radical)
            inout = d - Y
        else:
            inout = 1

    elif surf.type == "ellipsoid":
        center, axis, radii, rAxes = surf.params

        r = p - center
        rX = r.dot(axis)
        rY = r - (rX * axis + center)

        if (axis - rAxes[0]).length < 1e-5:
            radX, radY = radii
        else:
            radY, radY = radii

        radical = 1 - (rX / radX) ** 2
        if radical > 0:
            Y = radY * math.sqrt(radical)
            inout = rY - Y
        else:
            inout = 1

    elif surf.type == "cylinder_elliptic":
        center, axis, radii, rAxes = surf.params

        r = p - center
        X = r.dot(rAxes[1])
        Y = r.dot(rAxes[0])
        inout = (X / radii[1]) ** 2 + (Y / radii[0]) ** 2 - 1

        if surf.truncated and inout < 0:
            inout = btwPPlanes(p, center, axis)  # <0  between planes

    elif surf.type == "cylinder_hyperbolic":
        center, axis, radii, rAxes = surf.params

        r = p - center
        X = r.dot(rAxes[1])
        Y = r.dot(rAxes[0])
        inout = (X / radii[1]) ** 2 - (Y / radii[0]) ** 2 - 1

    elif surf.type == "paraboloid":
        center, axis, focal = surf.params

        r = p - center
        X = r.dot(axis)
        if X < 0:
            inout = 1
        else:
            v = r - X * axis
            d = v.length
            Y = math.sqrt(4 * focal * X)
            inout = d - Y

    elif surf.type == "torus":
        P, v, Ra, Rb, Rc = surf.params

        d = p - P
        z = d.dot(v)
        rz = d - z * v
        inout = (z / Rb) ** 2 + ((rz.length - Ra) / Rc) ** 2 - 1

    elif surf.type == "box":
        P, v1, v2, v3 = surf.params
        for v in (v1, v2, v3):
            inout = btwPPlanes(p, P, v)  # <0  between planes
            if inout > 0:
                break

    else:
        print(f"surface type {surf[0]} not considered")
        return

    return inout > 0


def btwPPlanes(p, p0, v):

    p1 = p0 + v
    inP0 = v.dot(p - p0)  # >0 plane(base plane) side inside the cylinder
    inP1 = v.dot(p - p1)  # >0 plane(base plane) side inside the cylinder

    if (inP0 > 0) and (inP1 < 0):
        return -1
    else:
        return 1
