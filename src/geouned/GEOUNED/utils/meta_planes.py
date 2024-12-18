import Part


def get_metaplanes(solid):
    planes = []
    for f in solid.Faces:
        if type(f.Surface) is Part.Plane:
            planes.append(f)

    metaplane_list = []
    for p in planes:
        loop = False
        for mp in metaplane_list:
            if p in mp:
                loop = True
        if loop:
            continue
        metap = [p]
        meta_loop([p], metap, planes)
        if len(metap) != 1:
            if no_convex(metap):
                metaplane_list.append(metap)
    return metaplane_list


def meta_loop(adjacents, metalist, planes):
    for p in adjacents:
        new_adjacents = metaplane(p, planes)
        for ap in reversed(new_adjacents):
            if ap in metalist:
                new_adjacents.remove(ap)
        metalist.extend(new_adjacents)
        meta_loop(new_adjacents, metalist, planes)


def no_convex(meta_plane_list):
    planes = meta_plane_list[:]
    while len(planes) > 1:
        p = planes.pop()
        Edges = p.OuterWire.Edges
        for e in Edges:
            try:
                type_curve = type(e.Curve)
            except:
                type_curve = None
            if type_curve is not Part.Line:
                continue
            adjacent_plane = other_face_edge(e, p, planes, outer_only=True)
            if adjacent_plane is not None:
                sign = region_sign(p, adjacent_plane, e)
                if sign == "AND":
                    return False
    return True


def convex_wire(p):
    Edges = p.OuterWire.Edges
    for e in Edges:
        if type(e.Curve) != Part.Line:
            return []

    normal = p.Surface.Axis
    v0 = Edges[0].Curve.Direction.cross(p.Surface.Axis)
    if Edges[0].Orientation == "Forward":
        v0 = -v0

    for e in Edges[1:]:
        v1 = e.Curve.Direction.cross(p.Surface.Axis)
        if e.Orientation == "Forward":
            v1 = -v1
        if normal.dot(v0.cross(v1)) < 0:
            return []
        v0 = v1

    return Edges


def metaplane(p, planes):
    Edges = p.OuterWire.Edges
    addplane = [p]
    for e in Edges:
        try:
            type_curve = type(e.Curve)
        except:
            type_curve = None
        if type_curve is not Part.Line:
            continue

        adjacent_plane = other_face_edge(e, p, planes, outer_only=True)
        if adjacent_plane is not None:
            sign = region_sign(p, adjacent_plane, e)
            if sign == "OR":
                addplane.append(adjacent_plane)
    return addplane


def region_sign(p1, p2, e1):
    normal1 = p1.Surface.Axis
    direction = e1.Curve.Direction
    if e1.Orientation == "Forward":
        direction = -direction
    vect = direction.cross(normal1)  # toward inside the face. For vect, p1 face orientation doesn't matter.

    normal2 = p2.Surface.Axis
    if p2.Orientation == "Forward":
        normal2 = -normal2

    return "OR" if vect.dot(normal2) < 0 else "AND"


def other_face_edge(current_edge, current_face, Faces, outer_only=False):
    for face in Faces:
        if face.isSame(current_face):
            continue

        Edges = face.OuterWire.Edges if outer_only else face.Edges
        for edge in Edges:
            if current_edge.isSame(edge):
                return face
