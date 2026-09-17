import math

# SplitBase, joinBase, SplitSolid, space_decomposition used to live here --
# moved to `geo.solid_ops`, 2026-09-17/18, shared with GEOUNED's own,
# previously near-identical copy in `build_region/build_region.py`/
# `build_region/splitFunction.py` (now deleted). See CLAUDE.md's
# "build_region/ vs CAD/buildSolidCell.py+splitFunction.py unification"
# entry. `surface_side` (below) is passed into the shared `SplitSolid` as
# its pluggable `classify` callable, unchanged -- this file's own exotic-
# quadric-aware point classification was deliberately not touched by that
# move.


def updateSurfacesValues(position, surfaces, knownSurf):
    position.update(knownSurf)
    sname = set(surfaces.keys())
    pname = set(position.keys())

    fullpos = position.copy()
    for name in sname.difference(pname):
        fullpos[name] = None
    return fullpos


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
        v = r - rX * rAxes[1]
        d = v.length

        one = 1 if onesht else -1
        radical = (rX / radii[1]) ** 2 + one

        if radical > 0:
            Y = radii[0] * math.sqrt(radical)
            inout = (d - Y) * one
        else:
            inout = one

    elif surf.type == "ellipsoid":
        center, axis, radii, rAxes = surf.params

        r = p - center
        rX = r.dot(axis)
        rY = (r - rX * axis).length

        if (axis - rAxes[0]).length < 1e-5:
            radX, radY = radii[0], radii[1]
        else:
            radX, radY = radii[1], radii[0]

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
        P, v, Ra, Rb, Rc, degenerated = surf.params
        if degenerated < 0:
            Ra = -Ra

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

    # bool(...): `inout` can be a numpy scalar (from GVector dot products /
    # lengths), so `inout > 0` may be numpy.bool_ rather than a plain
    # Python bool. GEOUNED's own BoolSequence.substitute() branches on
    # `type(val) is not bool`, which is True for numpy.bool_ -- it would
    # take the wrong branch (treating the value as another surface number
    # instead of a true/false substitution) and corrupt the sequence.
    return bool(inout > 0)


def btwPPlanes(p, p0, v):

    p1 = p0 + v
    inP0 = v.dot(p - p0)  # >0 plane(base plane) side inside the cylinder
    inP1 = v.dot(p - p1)  # >0 plane(base plane) side inside the cylinder

    if (inP0 > 0) and (inP1 < 0):
        return -1
    else:
        return 1
