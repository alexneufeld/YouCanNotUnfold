import FreeCAD
import Part
import networkx as nx
from FreeCAD import Vector, Matrix, Rotation, Placement
from math import radians, degrees
from enum import Enum, auto


# used when comparing positions in 3D space
eps = FreeCAD.Base.Precision.approximation()
# used when comparing angles
eps_angular = FreeCAD.Base.Precision.angular()


# Possible values of x in 'isinstance(some_sub_shape.Surface, x)'
PartSurface = (
    Part.SurfaceOfExtrusion
    | Part.Plane
    | Part.Cylinder
    | Part.BSplineSurface
    | Part.BezierSurface
    | Part.Cone
    | Part.Sphere
    | Part.SurfaceOfRevolution
    | Part.Toroid
)


def compare_plane_plane(p1: Part.Plane, p2: Part.Plane) -> bool:
    # returns True if the two planes have similar normals and the base
    # point of the first plane is (nearly) coincident with the second plane
    return (
        p1.Axis.isParallel(p2.Axis, eps_angular)
        and p1.Position.distanceToPlane(p2.Position, p2.Axis) < eps
    )


def compare_plane_cylinder(p: Part.Plane, c: Part.Cylinder) -> bool:
    # returns True if the cylinder is tangent to the plane
    # (there is 'line contact' between the surfaces)
    return (
        p.Axis.isNormal(c.Axis, eps_angular)
        and abs(abs(c.Center.distanceToPlane(p.Position, p.Axis)) - c.Radius) < eps
    )


def compare_cylinder_cylinder(c1: Part.Cylinder, c2: Part.Cylinder) -> bool:
    # returns True if the two cylinders have parallel axis' and those axis
    # are seperated by a distance of approximately r1 + r2
    return (
        c1.Axis.isParallel(c2.Axis, eps_angular)
        and abs(c1.Center.distanceToLine(c2.Center, c2.Axis) - (c1.Radius + c2.Radius))
        < eps
    )


def compare_plane_torus(p: Part.Plane, t: Part.Toroid) -> bool:
    return (
        p.Axis.isParallel(t.Axis, eps_angular)
        and abs(abs(t.Center.distanceToPlane(p.Position, p.Axis)) - t.MinorRadius) < eps
    )


def compare_cylinder_torus(c: Part.Cylinder, t: Part.Toroid) -> bool:
    return (
        c.Axis.isParallel(t.Axis, eps_angular)
        and c.Center.distanceToLine(t.Center, t.Axis) < eps
        and (
            abs(c.Radius - abs(t.MajorRadius - t.MinorRadius)) < eps
            or abs(c.Radius - abs(t.MajorRadius + t.MinorRadius)) < eps
        )
    ) or (
        c.Axis.isNormal(t.Axis, eps_angular)
        and abs(abs(t.Center.distanceToLine(c.Center, c.Axis)) - t.MajorRadius) < eps
        and abs(c.Radius - t.MinorRadius) < eps
    )


def compare_sphere_sphere(s1: Part.Sphere, s2: Part.Sphere) -> bool:
    return (
        s1.Center.distanceToPoint(s2.Center) < eps and abs(s1.Radius - s2.Radius) < eps
    )


def compare_plane_sphere(p: Part.Plane, s: Part.Sphere) -> bool:
    return abs(abs(s.Center.distanceToPlane(p.Position, p.Axis)) - s.radius) < eps


def compare_cylinder_sphere(c: Part.Cylinder, s: Part.Sphere) -> bool:
    return (
        s.Center.distanceToLine(c.Center, c.Axis) < eps
        and abs(s.Radius - c.Radius) < eps
    )


def is_faces_tangent(f1: Part.Face, f2: Part.Face) -> bool:
    match f1.Surface.TypeId:
        case "Part::GeomPlane":
            match f2.Surface.TypeId:
                case "Part::GeomPlane":
                    return compare_plane_plane(f1.Surface, f2.Surface)
                case "Part::GeomCylinder":
                    return compare_plane_cylinder(f1.Surface, f2.Surface)
                case "Part::GeomToroid":
                    return compare_plane_torus(f1.Surface, f2.Surface)
                case "Part::GeomSphere":
                    return compare_plane_sphere(f1.Surface, f2.Surface)
                case "Part::GeomSurfaceOfExtrusion":
                    return False
                case "Part::GeomCone":
                    return False
                case _:
                    return False
        case "Part::GeomCylinder":
            match f2.Surface.TypeId:
                case "Part::GeomPlane":
                    return compare_plane_cylinder(f2.Surface, f1.Surface)
                case "Part::GeomCylinder":
                    return compare_cylinder_cylinder(f1.Surface, f2.Surface)
                case "Part::GeomToroid":
                    return False
                case "Part::GeomSphere":
                    return compare_cylinder_sphere(f1.Surface, f2.Surface)
                case "Part::GeomSurfaceOfExtrusion":
                    return False
                case "Part::GeomCone":
                    return False
                case _:
                    return False
        case "Part::GeomToroid":
            match f2.Surface.TypeId:
                case "Part::GeomPlane":
                    return compare_plane_torus(f2.Surface, f1.Surface)
                case "Part::GeomCylinder":
                    return compare_cylinder_torus(f2.Surface, f1.Surface)
                case "Part::GeomToroid":
                    return False
                case "Part::GeomSphere":
                    return False
                case "Part::GeomSurfaceOfExtrusion":
                    return False
                case "Part::GeomCone":
                    return False
                case _:
                    return False
        case "Part::GeomSphere":
            match f2.Surface.TypeId:
                case "Part::GeomPlane":
                    return compare_plane_sphere(f2.Surface, f1.Surface)
                case "Part::GeomCylinder":
                    return compare_cylinder_sphere(f2.Surface, f1.Surface)
                case "Part::GeomToroid":
                    return False
                case "Part::GeomSphere":
                    return compare_sphere_sphere(f1.Surface, f2.Surface)
                case "Part::GeomSurfaceOfExtrusion":
                    return False
                case "Part::GeomCone":
                    return False
                case _:
                    return False
        case "Part::GeomSurfaceOfExtrusion":
            match f2.Surface.TypeId:
                case "Part::GeomPlane":
                    return False
                case "Part::GeomCylinder":
                    return False
                case "Part::GeomToroid":
                    return False
                case "Part::GeomSphere":
                    return False
                case "Part::GeomSurfaceOfExtrusion":
                    return False
                case "Part::GeomCone":
                    return False
                case _:
                    return False
        case "Part::GeomCone":
            match f2.Surface.TypeId:
                case "Part::GeomPlane":
                    return False
                case "Part::GeomCylinder":
                    return False
                case "Part::GeomToroid":
                    return False
                case "Part::GeomSphere":
                    return False
                case "Part::GeomSurfaceOfExtrusion":
                    return False
                case "Part::GeomCone":
                    return False
                case _:
                    return False
        case _:
            return False


def build_graph_of_tangent_faces(shp: Part.Shape, root: int):
    # created a simple undirected graph object
    gr = nx.Graph()

    # weird shape mutability nonsense? -> have to use indexes, because the underlying geometry may get changed around while building the graph

    face_hashes = [f.hashCode() for f in shp.Faces]
    index_lookup = {h: i for i, h in enumerate(face_hashes)}

    # get pairs of faces that share the same edge
    candidates = [
        (i, shp.ancestorsOfType(e, Part.Face)) for i, e in enumerate(shp.Edges)
    ]
    # filter to remove seams on cylinders or other faces that wrap back onto themselves
    # other than self-adjacent faces, edges should always have 2 face ancestors
    # this assumption is probably only valid for watertight solids.
    for edge_index, faces in filter(lambda c: len(c[1]) == 2, candidates):
        face_a, face_b = faces
        if is_faces_tangent(face_a, face_b):
            gr.add_edge(
                index_lookup[face_a.hashCode()],
                index_lookup[face_b.hashCode()],
                label=edge_index,
            )
    for c in nx.connected_components(gr):
        if root in c:
            return gr.subgraph(c).copy()
    raise RuntimeError(
        "Couldn't find a network of useable faces from the chosen seed face"
    )


class UVRef(Enum):
    # reference corner for a rectangular-ish surface patch
    BOTTOM_LEFT = auto()
    BOTTOM_RIGHT = auto()
    TOP_LEFT = auto()
    TOP_RIGHT = auto()


def unroll_cylinder(
    cylindrical_face: Part.Face, refpos: UVRef, k_factor: float, thickness: float
) -> Part.Face:
    # the u,v reference position becomes x,y = 0,0
    # the face is oriented z-up
    #
    # the u period is always positive: 0.0 <= umin < umax <= 2*pi
    umin, umax, vmin, vmax = cylindrical_face.ParameterRange
    bend_angle = umax - umin
    radius = cylindrical_face.Surface.Radius
    bend_allowance = (radius + k_factor * thickness) * bend_angle
    # y_scale_factor = bend_allowance / (radius * bend_angle)
    # dummy implementation - just return a square patch
    x_length = max(
        [e.Length for e in cylindrical_face.Edges if e.Curve.TypeId == "Part::GeomLine"]
    )
    v1 = Vector(0, 0)
    v2 = Vector(x_length, 0)
    v3 = Vector(x_length, bend_allowance)
    v4 = Vector(0, bend_allowance)
    l1 = Part.makeLine(v1, v2)
    l2 = Part.makeLine(v2, v3)
    l3 = Part.makeLine(v3, v4)
    l4 = Part.makeLine(v4, v1)
    return Part.makeFace(Part.Wire([l1, l2, l3, l4]), "Part::FaceMakerSimple")


def compute_unbend_transform(
    p1: Part.Face, p2: Part.Face, bc: Part.Face, base_edge: Part.Edge
) -> Matrix:
    # angle = p1.Surface.Axis.getAngle(p2.Surface.Axis)
    # for cylindrical surfaces, the u-parameter corresponds to the radial direction, and the u-period is the radial boundary of the cylindrical patch. The v-period corresponds to the axial direction.
    umin, umax, vmin, vmax = bc.ParameterRange
    # the u period is always positive: 0.0 <= umin < umax <= 2*pi
    bend_angle = umax - umin
    radius = bc.Surface.Radius
    # disallow fully cylindrical bends. These can't be formed because the opposite edge of the sheet will intersect p1. Though, if this error occurs, the most probable cause is bad input geometry
    if bend_angle > radians(359.9):
        raise RuntimeError("Bend angle must be less that 359.9 degrees")
    # for cylindrical surfaces:
    # the surface of the outside of a tube ('convex') is always forward-oriented
    # the surface of the inside of a tube ('concave') is always reverse-oriented
    # bend_direction = {"Reversed": "Up", "Forward": "Down"}[bc.Orientation]
    # the reference edge should intersect with the bent cylindrical surface at either the start or end of the cylinders u-parameter range. We need to determine wich of these possibilities is correct
    first_corner_point = bc.valueAt(umin, vmin)
    second_corner_point = bc.valueAt(umax, vmin)
    # at least one of these points should be on the starting edge
    dist1 = first_corner_point.distanceToLine(
        base_edge.Curve.Location, base_edge.Curve.Direction
    )
    dist2 = second_corner_point.distanceToLine(
        base_edge.Curve.Location, base_edge.Curve.Direction
    )
    if dist1 < eps:
        cylinder_orientation = "Forward"
    elif dist2 < eps:
        cylinder_orientation = "Reverse"
    else:
        raise RuntimeError("no point on reference edge")
    # the x-axis of our desired reference is the tangent vector to a radial
    # line on the cylindrical surface, pointing away from the p1 face.
    # We can compute this curve from the cylindrical surface
    if cylinder_orientation == "Forward":
        tangent_vector, binormal_vector = bc.Surface.tangent(umin, vmin)
        y_axis = tangent_vector
        # use the normal of the face and not the surface here
        # If the face is reverse oriented, the surface normal will be flipped relative to the face normal
        z_axis = bc.normalAt(umin, vmin)
    else:
        tangent_vector, binormal_vector = bc.Surface.tangent(umax, vmin)
        y_axis = tangent_vector.negative()
        z_axis = bc.normalAt(umax, vmin)
    # place the reference point such that the cylindrical face lies in the
    # (+x, +y) quadrant of the xy-plane of the reference coordinate system
    x_axis = y_axis.cross(z_axis)
    if cylinder_orientation == "Forward":
        if x_axis.dot(corner_1 := bc.valueAt(umin, vmin)) < x_axis.dot(
            corner_2 := bc.valueAt(umin, vmax)
        ):
            lcs_base_point = corner_1
        else:
            lcs_base_point = corner_2
    else:
        if x_axis.dot(corner_1 := bc.valueAt(umax, vmin)) < x_axis.dot(
            corner_2 := bc.valueAt(umax, vmax)
        ):
            lcs_base_point = corner_1
        else:
            lcs_base_point = corner_2
    # the x-axis can be left as a 0-vector, it will be ignored based on the axis priority string
    lcs_rotation = Rotation(Vector(), y_axis, z_axis, "ZYX")
    alignment_transform = Placement(lcs_base_point, lcs_rotation).toMatrix()
    # the actual unbend transformation is found by reversing the rotation of
    # the p2 face due to bending, then pushing it forward according to the bend allowance
    k_factor = 0.5
    thickness = 2.0
    bend_allowance = (radius + k_factor * thickness) * bend_angle
    allowance_transform = Matrix(
        1,
        0,
        0,
        0,
        0,
        1,
        0,
        bend_allowance,
        0,
        0,
        1,
        0,
        0,
        0,
        0,
        1,
    )

    rot = Rotation(Vector(1, 0, 0), -1 * degrees(bend_angle)).toMatrix()
    translate = Matrix(
        1,
        0,
        0,
        0,
        0,
        1,
        0,
        0,
        0,
        0,
        1,
        radius,
        0,
        0,
        0,
        1,
    )

    # compose transformations to get the final matrix
    mat = Matrix()
    mat.transform(Vector(), alignment_transform.inverse())
    mat.transform(Vector(), translate * rot * translate.inverse())
    mat.transform(Vector(), allowance_transform)
    mat.transform(Vector(), alignment_transform)

    Part.show(p2.transformed(mat), "wtf")

    unrolled_face = unroll_cylinder(bc, UVRef.BOTTOM_LEFT, k_factor, thickness)
    Part.show(unrolled_face.transformed(alignment_transform))

    return mat


def unfold(shape: Part.Shape, root_face_index: int, k_factor: int = 0.5) -> Part.Shape:
    graph_of_sheet_faces = build_graph_of_tangent_faces(shp, root_face)

    # we could also get a random spanning tree here. Would that be faster?
    # Or is it better to take the opportunity to get a spanning tree that meets
    # some criteria for minimization?
    # I.E.: the shorter the longest path in the tree, the fewer nested
    # transformations we have to compute
    spanning_tree = nx.minimum_spanning_tree(graph_of_sheet_faces, weight="label")

    # convert to 'directed tree'
    dg = nx.DiGraph()
    for node in spanning_tree:
        # color the nodes nicely (for debugging)
        dg.add_node(
            node,
            color={
                "Part::GeomPlane": "red",
                "Part::GeomCylinder": "blue",
            }[shp.Faces[node].Surface.TypeId],
        )

    lengths = nx.all_pairs_shortest_path_length(spanning_tree)
    ll = {k: kv for k, kv in lengths}
    to_root = ll[root_face]

    for f1, f2, edata in spanning_tree.edges(data=True):
        if to_root[f1] <= to_root[f2]:
            dg.add_edge(f1, f2, label=edata["label"])
        else:
            dg.add_edge(f2, f1, label=edata["label"])

    assert nx.is_directed_acyclic_graph(dg)

    print(str(nx.nx_pydot.to_pydot(dg)))

    # the digraph should now have everything we need to unfold the shape
    # unfold bends adjacent to 2 planar faces
    for e in [
        e for e in dg.edges if shp.Faces[e[0]].Surface.TypeId == "Part::GeomCylinder"
    ]:
        bend_part = shp.Faces[e[0]]
        flat_part = shp.Faces[e[1]]
        face_before_bend_index = next(dg.predecessors(e[0]))
        face_before_bend = shp.Faces[face_before_bend_index]
        edge_before_bend_index = dg.get_edge_data(face_before_bend_index, e[0])["label"]
        edge_before_bend = shp.Edges[edge_before_bend_index]
        compute_unbend_transform(
            face_before_bend, flat_part, bend_part, edge_before_bend
        )
        print(f"{face_before_bend_index=},{edge_before_bend_index=}")

    return Part.Shape()


if __name__ == "__main__":
    selection_object = FreeCAD.Gui.Selection.getCompleteSelection()[0]
    shp = selection_object.Object.Shape
    root_face = int(selection_object.SubElementNames[0][4:]) - 1
    _ = unfold(shp, root_face)
