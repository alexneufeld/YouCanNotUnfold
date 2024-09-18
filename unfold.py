import FreeCAD
import Part
import networkx as nx
from FreeCAD import Vector, Matrix, Rotation, Placement
from math import sin, cos, radians, degrees
from enum import Enum, auto


eps = (
    FreeCAD.Base.Precision.approximation()
)  # used when comapring positions in 3D space
eps_angular = FreeCAD.Base.Precision.angular()  # used when comparing angles


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
    # returns True if the two planes have similar normals and the base point of the first plane is (nearly) coincident with the second plane
    return (
        p1.Axis.isParallel(p2.Axis, eps_angular)
        and p1.Position.distanceToPlane(p2.Position, p2.Axis) < eps
    )


def compare_plane_cylinder(p: Part.Plane, c: Part.Cylinder) -> bool:
    # returns True if the cylinder is tangent to the plane (there is 'line contact' between the surfaces)
    return (
        p.Axis.isNormal(c.Axis, eps_angular)
        and abs(abs(c.Center.distanceToPlane(p.Position, p.Axis)) - c.Radius) < eps
    )


def compare_cylinder_cylinder(c1: Part.Cylinder, c2: Part.Cylinder) -> bool:
    # returns True if the two cylinders have parallel axis' and those axis are seperated by a distance of approximately r1 + r2
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
        and abs(c.Radius - abs(t.MajorRadius - t.MinorRadius)) < eps
    ) or (
        c.Axis.isNormal(t.Axis, eps_angular)
        and abs(t.Center.distanceToLine(c.Center, c.Axis)) < eps
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
    # if both faces are planar, check that they have similar plane surfaces
    #      ┌───────┐
    #      │       │
    #    ┌───────────┐
    # ┌─ │   Plane   │ ─┐
    # │  └───────────┘  │
    # │    │            │
    # │    │            │
    # │    │            │
    # │  ┌───────────┐  │
    # │  │   Cone    │ ─┼────┐
    # │  └───────────┘  │    │
    # │    │            │    │
    # │    │            │    │
    # │    │            │    │
    # │  ┌───────────┐  │    │
    # │  │ Cylinder  │ ─┘    │
    # │  └───────────┘       │
    # │    │                 │
    # │    │                 │
    # │    │                 │
    # │  ┌───────────┐       │
    # └─ │ Extrusion │ ──────┘
    #    └───────────┘
    #      │       │
    #      └───────┘

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

    #
    # if isinstance(f1.Surface, Part.Plane):
    #     if isinstance(f2.Surface, Part.Plane):
    #         return compare_plane_plane(f1.Surface, f2.Surface)
    #     if isinstance(f2.Surface, Part.Cylinder):
    #         return compare_plane_cylinder(f1.Surface, f2.Surface)
    # if isinstance(f1.Surface, Part.Cylinder):
    #     if isinstance(f2.Surface, Part.Plane):
    #         return compare_plane_cylinder(f2.Surface, f1.Surface)
    # return False


# class node:
#     def __init__(self, index: int, shp: Part.Face):
#         self.index = index
#         self.shp = shp
#
#
# class edge:
#     def __init__(self, n1: node, n2: node):
#         self.nodes = {n1, n2}  # undirected edge
#
#
# def faces_have_shared_edge(f1: Part.Face, f2: Part.Face) -> bool:
#     return bool({x.hashCode() for x in f1.Edges} & {x.hashCode() for x in f2.Edges})

#
# class graph:
#     def __init__(self, nodes: list[node], edges: list[edge]):
#         self.nodes = nodes
#         self.edges = edges
#
#     def __repr__(self):
#         return f"<graph with {len(self.nodes)} nodes and {len(self.edges)} edges>"
#
#     @classmethod
#     def from_shape(cls, shp: Part.Shape) -> Self:
#         list_of_nodes = []
#         for i, face in enumerate(shp.Faces):
#             list_of_nodes.append(node(i, face))
#         list_of_edges = []
#
#         candidates = [shp.ancestorsOfType(e, Part.Face) for e in shp.Edges]
#         face_hashes = [f.hashCode() for f in shp.Faces]
#         index_lookup = {h: i for i, h in enumerate(face_hashes)}
#         # filter to remove seams on cylinders or other faces that wrap back onto themselves
#         for face_a, face_b in filter(lambda c: len(c) == 2, candidates):
#             if is_faces_tangent(face_a, face_b):
#                 node1 = list_of_nodes[index_lookup[face_a.hashCode()]]
#                 node2 = list_of_nodes[index_lookup[face_b.hashCode()]]
#                 list_of_edges.append(edge(node1, node2))
#
#         # for n1, n2 in combinations(list_of_nodes, 2):
#         #     if faces_have_shared_edge(n1.shp, n2.shp) and is_faces_tangent(n1.shp, n2.shp):
#         #             list_of_edges.append(edge(n1, n2))
#         return cls(list_of_nodes, list_of_edges)
#
#     def to_graphviz(self) -> str:
#         # get connected components of the graph
#         gr = nx.Graph()
#         for e in self.edges:
#             gr.add_edge(tuple(e.nodes)[0].index, tuple(e.nodes)[1].index)
#         components = list(nx.connected_components(gr))
#         # print({n: sum(1 for c in nx.find_cliques(gr) if n in c) for n in gr})
#         # print(components)
#         graph_coloring = []
#         for color, component in zip(cycle(range(1, 11)), components):
#             for n in component:
#                 graph_coloring.append(f"    {n} [color={color}];\n")
#
#         # return nx.nx_pydot.to_pydot(gr)
#
#         return (
#             "graph {\n"
#             + "    node [colorscheme=spectral10];\n"
#             + "".join(graph_coloring)
#             + "".join(
#                 [
#                     f"    {tuple(e.nodes)[0].index} -- {tuple(e.nodes)[1].index};\n"
#                     for e in self.edges
#                 ]
#             )
#             + "}\n"
#         )


def fast_nx_graph_build(shp: Part.Shape, root: int):
    # created a simple undirected graph object
    gr = nx.Graph()
    # faces are hashable objects that can be used as nodes directly
    # (the .__hash__() method and also equivalent .hashCode() are available for shapes)
    # we don't even have to add the shape's faces to the graph, nodes are implicitly
    # created along with edges

    # weird shape mutability nonsense? -> have to use indexes

    face_hashes = [f.hashCode() for f in shp.Faces]
    index_lookup = {h: i for i, h in enumerate(face_hashes)}

    # get pairs of faces that share the same edge
    candidates = [
        (i, shp.ancestorsOfType(e, Part.Face)) for i, e in enumerate(shp.Edges)
    ]
    # from pprint import pprint
    # pprint(candidates)
    # filter to remove seams on cylinders or other faces that wrap back onto themselves
    # other than self-adjacent faces, edges should always have 2 face ancestors
    # this assumption is only valid for watertight solids though
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


# print(graph.from_shape(FreeCAD.Gui.Selection.getSelection()[0].Shape).to_graphviz())
# with open("/home/alex/test.dot", 'w') as f:
selection_object = FreeCAD.Gui.Selection.getCompleteSelection()[0]
shp = selection_object.Object.Shape
root_face = int(selection_object.SubElementNames[0][4:]) - 1
# root_face = shp.Faces[67]
# root_face = 67
graph_of_sheet_faces = fast_nx_graph_build(shp, root_face)
# use the edge labels (shape indices) as the weight for finding the spanning tree
# is the code still deterministic if no weights are provided? Is it faster?
spanning_tree = nx.minimum_spanning_tree(graph_of_sheet_faces, weight="label")
# is this faster?
# spanning_tree = nx.random_spanning_tree(graph_of_sheet_faces, seed=shp.hashCode())
# spanning_tree = T = nx.minimum_spanning_arborescence(graph_of_sheet_faces, attr="label")  # does not work on non-directed graphs
# print(str(nx.nx_pydot.to_pydot(spanning_tree)))

# convert to directed tree
dg = nx.DiGraph()
for node in spanning_tree:
    # color the nodes nicely (for debugging)
    # print(f"{node=}")
    dg.add_node(
        node,
        color={
            "Part::GeomPlane": "red",
            "Part::GeomCylinder": "blue",
        }[shp.Faces[node].Surface.TypeId],
    )
# dg.add_nodes_from(spanning_tree.nodes)
lengths = nx.all_pairs_shortest_path_length(spanning_tree)
ll = {k: kv for k, kv in lengths}
to_root = ll[root_face]
# from pprint import pprint
for f1, f2, edata in spanning_tree.edges(data=True):
    if to_root[f1] <= to_root[f2]:
        dg.add_edge(f1, f2, label=edata["label"])
    else:
        dg.add_edge(f2, f1, label=edata["label"])

assert nx.is_directed_acyclic_graph(dg)

print(str(nx.nx_pydot.to_pydot(dg)))


class UVRef(Enum):
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


#  "a.cross(b)"
#
#      ┌─────────────────┐
#      │        a        │
#      ├──┬──┬──┬──┬──┬──┤
#      │ x│ y│ z│-x│-y│-z│
# ┌─┬──┼──┼──┼──┼──┼──┼──┤
# │ │ x│ 0│-z│ y│ 0│ z│-y│
# │ ├──┼──┼──┼──┼──┼──┼──┤
# │ │ y│ z│ 0│-x│-z│ 0│ x│
# │ ├──┼──┼──┼──┼──┼──┼──┤
# │ │ z│-y│ x│ 0│ y│-x│ 0│
# │b├──┼──┼──┼──┼──┼──┼──┤
# │ │-x│ 0│ z│-y│ 0│-z│ y│
# │ ├──┼──┼──┼──┼──┼──┼──┤
# │ │-y│-z│ 0│ x│ z│ 0│-x│
# │ ├──┼──┼──┼──┼──┼──┼──┤
# │ │-z│ y│-x│ 0│-y│ x│ 0│
# └─┴──┴──┴──┴──┴──┴──┴──┘


def compute_unbend_transform(
    p1: Part.Face, p2: Part.Face, bc: Part.Face, base_edge: Part.Edge
) -> Matrix:
    assert isinstance(p1.Surface, Part.Plane)
    assert isinstance(p2.Surface, Part.Plane)
    assert isinstance(base_edge.Curve, Part.Line)
    assert isinstance(bc.Surface, Part.Cylinder)
    # angle = p1.Surface.Axis.getAngle(p2.Surface.Axis)
    # for cylindrical surfaces, the u-parameter corresponds to the radial direction, and the u-period is the radial boundary of the sylindrical patch. The v-period corresponds to the axial direction.
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
    # the x-axis of our desired reference is the tangent vector to a radial line on the cylindrical surface, pointing away from the p1 face. We can compute this curve from the cylindrical surface
    if cylinder_orientation == "Forward":
        tangent_vector, binormal_vector = bc.Surface.tangent(umin, vmin)
        y_axis = tangent_vector
        z_axis = bc.normalAt(umin, vmin)
    else:
        tangent_vector, binormal_vector = bc.Surface.tangent(umax, vmin)
        y_axis = tangent_vector.negative()
        z_axis = bc.normalAt(umax, vmin)
    # use the normal of the face and not the surface here - if the face is reverse oriented, the surface normal will be flipped relative to the face normal
    # if bc.Orientation == "Reversed":
    #     z_axis = z_axis*-1
    # place the reference point such that the cylindrical face lies in the (+x, +y) quadrant of the xy-plane
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
    # lcs_base_point = base_edge.CenterOfGravity

    # y_vector_possibly_woring_direction = base_edge.Curve.Direction
    # the z-axis of our desired reference coordinate system is the normal vector of the stationary planar face:
    # don't rely on p1.Surface.Axis here, as it may be flipped relative to the actual face
    # z_axis = p1.Surface.Axis
    # z_axis = p1.normalAt(0,0)  # dummy uv coords should be fine for planar faces
    # z_axis =
    # we have a handy rotation constructer that will build the correct matrix for us from the known x and z vectors
    # the y-axis can be left as a 0-vector, it will be ignored based on the "XZY" priority string
    lcs_rotation = Rotation(Vector(), y_axis, z_axis, "ZYX")
    alignment_transform = Placement(lcs_base_point, lcs_rotation).toMatrix()
    # the actual unbend transformation is found by reversing the rotation of the p2 face sue to bending, then pushing it forward according to the bend allowance
    k_factor = 0.5
    thickness = 2.0
    bend_allowance = (radius + k_factor * thickness) * bend_angle
    # this corresponds to a rotaion about a copy of the y-axis, translated in the z direction by the value of the bend radius
    unrotate_transform = Matrix(
        cos(bend_angle),
        0,
        -1 * sin(bend_angle),
        radius * sin(bend_angle),
        0,
        1,
        0,
        0,
        sin(bend_angle),
        0,
        cos(bend_angle),
        -1 * radius * cos(bend_angle) + radius,
        0,
        0,
        0,
        1,
    )
    # this one is just a translation forward in the x-direction
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
    inverse_align = alignment_transform.inverse()
    overall_transform = (
        inverse_align * unrotate_transform * allowance_transform * alignment_transform
    )
    obj = FreeCAD.ActiveDocument.addObject("PartDesign::CoordinateSystem", "test_cs")
    obj.Placement = Placement(alignment_transform)
    # Part.show(p2.transformed(inverse_align, True), "aligned_to_origin")
    # Part.show(Part.Vertex(lcs_base_point + z_axis.normalize()*radius), "rotaion_base_vertex")
    # Part.show(Part.Vertex(lcs_base_point + z_axis.normalize()*radius + y_axis.cross(z_axis)), "rotation_dir_vertex")

    # for i in range(10):
    #     obj = FreeCAD.ActiveDocument.addObject('PartDesign::CoordinateSystem',f'aligned_cs_{i}')
    #     rot = Rotation(Vector(1,0,0),degrees(bend_angle)*(i+1)/10).toMatrix()
    #     translate = Matrix(
    #         1,0,0,0,
    #         0,1,0,0,
    #         0,0,1,radius,
    #         0,0,0,1,
    #     )
    #     aligned_pl = Placement(alignment_transform*translate*rot*translate.inverse())
    #     obj.Placement = aligned_pl

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

    subtransform_1 = translate * rot * translate.inverse()

    test_mat = Matrix()
    test_mat.transform(Vector(), alignment_transform.inverse())
    test_mat.transform(Vector(), subtransform_1)
    test_mat.transform(Vector(), allowance_transform)
    test_mat.transform(Vector(), alignment_transform)

    Part.show(p2.transformed(test_mat), "wtf")

    # Part.show(p2.transformed((alignment_transform.inverse()*subtransform_1)*allowance_transform))
    # Part.show(p2.transformed(translate*rot*translate.inverse()*alignment_transform))

    # aligned_pl.rotate(z_axis.normalize()*radius, y_axis.cross(z_axis), degrees(bend_angle))
    # Part.show(p2.transformed(inverse_align*unrotate_transform, True), "rotated")
    # Part.show(p2.transformed(inverse_align*unrotate_transform*allowance_transform, True), "with_allowance")

    # Part.show(p2.transformed(overall_transform), "overall")

    # also show off the unrolled cylindrical patch:
    unrolled_face = unroll_cylinder(bc, UVRef.BOTTOM_LEFT, k_factor, thickness)
    Part.show(unrolled_face.transformed(alignment_transform))

    return overall_transform


# the digraph should now have everything we need to unfold the shape
# unfold bends adjacent to 2 planar faces
for e in [
    e for e in dg.edges if shp.Faces[e[0]].Surface.TypeId == "Part::GeomCylinder"
]:
    bend_part = shp.Faces[e[0]]
    flat_part = shp.Faces[e[1]]
    face_before_bend_index = next(dg.predecessors(e[0]))
    face_before_bend = shp.Faces[face_before_bend_index]
    # edge_before_bend =  parent_graph_edge[2]["label"]
    edge_before_bend_index = dg.get_edge_data(face_before_bend_index, e[0])["label"]
    edge_before_bend = shp.Edges[edge_before_bend_index]
    compute_unbend_transform(face_before_bend, flat_part, bend_part, edge_before_bend)
    print(f"{face_before_bend_index=},{edge_before_bend_index=}")
