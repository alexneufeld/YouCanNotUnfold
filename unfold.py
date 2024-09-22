import FreeCAD
import Part
import networkx as nx
from FreeCAD import Vector, Matrix, Rotation, Placement
from math import radians, degrees, log10
from enum import Enum, auto
from statistics import mode
from itertools import combinations


# used when comparing positions in 3D space
eps = FreeCAD.Base.Precision.approximation()
# used when comparing angles
eps_angular = FreeCAD.Base.Precision.angular()


def round_vector(vec: FreeCAD.Vector, ndigits: int = None) -> FreeCAD.Vector:
    return FreeCAD.Vector(*[round(d, ndigits) for d in vec])


def estimate_thickness_from_cylinders(shp: Part.Shape) -> float:
    num_places = abs(int(log10(eps)))
    curv_map = {}
    for face in shp.Faces:
        if isinstance(face.Surface, Part.Cylinder):
            # normalize the axis and centerpoint
            normalized_axis = face.Surface.Axis.normalize()
            if normalized_axis.dot(FreeCAD.Vector(0, 0, -1)) < 0:
                normalized_axis = normalized_axis.negative()
            cleaned_axis = round_vector(normalized_axis, num_places)
            adjusted_center = face.Surface.Center.projectToPlane(
                FreeCAD.Vector(), normalized_axis
            )
            cleaned_center = round_vector(adjusted_center, num_places)
            key = (*cleaned_axis, *cleaned_center)
            if key in curv_map:
                curv_map[key].append(abs(face.Surface.Radius))
            else:
                curv_map[key] = [
                    face.Surface.Radius,
                ]
    combined_list_of_thicknesses = []
    for radset in curv_map.values():
        if len(radset) > 1:
            for r1, r2 in combinations(radset, 2):
                if (val := abs(r1 - r2)) > eps:
                    combined_list_of_thicknesses.append(val)
    return mode(combined_list_of_thicknesses)


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
    y_scale_factor = radius * bend_allowance / (radius * bend_angle)
    print(f"{y_scale_factor=}")
    wire = Part.Wire()
    for e in cylindrical_face.Edges:
        edge_on_surface, e_param_min, e_param_max = cylindrical_face.curveOnSurface(e)
        # TODO: fix bspline impl
        if isinstance(edge_on_surface, Part.Geom2d.Line2d):
            v1 = edge_on_surface.value(e_param_min)
            y1, x1 = v1.x - umin, v1.y - vmin
            v2 = edge_on_surface.value(e_param_max)
            y2, x2 = v2.x - umin, v2.y - vmin
            line = Part.makeLine(
                Vector(x1, y1 * y_scale_factor), Vector(x2, y2 * y_scale_factor)
            )
            wire.add(line)
            # Part.show(line, 'line_segment')
        elif isinstance(edge_on_surface, Part.Geom2d.BSplineCurve2d):
            poles_and_weights = edge_on_surface.getPolesAndWeights()
            poles = [
                (v - vmin, (u - umin) * y_scale_factor, 0)
                for u, v, _ in poles_and_weights
            ]
            weights = [w for _, _, w in poles_and_weights]
            spline = Part.BSplineCurve()
            spline.buildFromPolesMultsKnots(poles=poles, weights=weights)
            wire.add(spline.toShape())
            # Part.show(spline.toShape(), 'line_segment')
        else:
            raise ValueError(
                f"Unhandled curve type when unfolding face: {type(edge_on_surface)}"
            )

    return Part.makeFace(wire, "Part::FaceMakerBullseye")


def compute_unbend_transform(
    bent_face: Part.Face, base_edge: Part.Edge, thickness: float, k_factor: float
) -> Matrix:
    # for cylindrical surfaces, the u-parameter corresponds to the radial direction, and the u-period is the radial boundary of the cylindrical patch. The v-period corresponds to the axial direction.
    umin, umax, vmin, vmax = bent_face.ParameterRange
    # the u period is always positive: 0.0 <= umin < umax <= 2*pi
    bend_angle = umax - umin
    radius = bent_face.Surface.Radius
    # disallow fully cylindrical bends. These can't be formed because the opposite edge of the sheet will intersect p1. Though, if this error occurs, the most probable cause is bad input geometry
    if bend_angle > radians(359.9):
        raise RuntimeError("Bend angle must be less that 359.9 degrees")
    # for cylindrical surfaces:
    # the surface of the outside of a tube ('convex') is always forward-oriented
    # the surface of the inside of a tube ('concave') is always reverse-oriented
    # bend_direction = {"Reversed": "Up", "Forward": "Down"}[bent_face.Orientation]
    # the reference edge should intersect with the bent cylindrical surface at either the start or end of the cylinders u-parameter range. We need to determine wich of these possibilities is correct
    first_corner_point = bent_face.valueAt(umin, vmin)
    second_corner_point = bent_face.valueAt(umax, vmin)
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
        tangent_vector, binormal_vector = bent_face.Surface.tangent(umin, vmin)
        y_axis = tangent_vector
        # use the normal of the face and not the surface here
        # If the face is reverse oriented, the surface normal will be flipped relative to the face normal
        z_axis = bent_face.normalAt(umin, vmin)
    else:
        tangent_vector, binormal_vector = bent_face.Surface.tangent(umax, vmin)
        y_axis = tangent_vector.negative()
        z_axis = bent_face.normalAt(umax, vmin)
    # place the reference point such that the cylindrical face lies in the
    # (+x, +y) quadrant of the xy-plane of the reference coordinate system
    x_axis = y_axis.cross(z_axis)
    if cylinder_orientation == "Forward":
        if x_axis.dot(corner_1 := bent_face.valueAt(umin, vmin)) < x_axis.dot(
            corner_2 := bent_face.valueAt(umin, vmax)
        ):
            lcs_base_point = corner_1
        else:
            lcs_base_point = corner_2
    else:
        if x_axis.dot(corner_1 := bent_face.valueAt(umax, vmin)) < x_axis.dot(
            corner_2 := bent_face.valueAt(umax, vmax)
        ):
            lcs_base_point = corner_1
        else:
            lcs_base_point = corner_2
    # the x-axis can be left as a 0-vector, it will be ignored based on the axis priority string
    lcs_rotation = Rotation(Vector(), y_axis, z_axis, "ZYX")
    alignment_transform = Placement(lcs_base_point, lcs_rotation).toMatrix()
    # the actual unbend transformation is found by reversing the rotation of
    # the p2 face due to bending, then pushing it forward according to the bend allowance
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

    return mat


def unfold(shape: Part.Shape, root_face_index: int, k_factor: int = 0.5) -> Part.Shape:
    graph_of_sheet_faces = build_graph_of_tangent_faces(shp, root_face)

    thickness = estimate_thickness_from_cylinders(shape)

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
        e for e in dg.edges if shp.Faces[e[1]].Surface.TypeId == "Part::GeomCylinder"
    ]:
        print(e)
        bend_part = shp.Faces[e[1]]
        edge_before_bend_index = dg.get_edge_data(e[0], e[1])["label"]
        edge_before_bend = shp.Edges[edge_before_bend_index]
        _ = compute_unbend_transform(bend_part, edge_before_bend, thickness, k_factor)
        _ = unroll_cylinder(bend_part, UVRef.BOTTOM_LEFT, k_factor, thickness)

    return Part.Shape()


if __name__ == "__main__":
    selection_object = FreeCAD.Gui.Selection.getCompleteSelection()[0]
    shp = selection_object.Object.Shape
    root_face = int(selection_object.SubElementNames[0][4:]) - 1
    _ = unfold(shp, root_face, k_factor=0.5)
