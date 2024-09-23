import FreeCAD
import Part
import networkx as nx
from FreeCAD import Vector, Matrix, Rotation, Placement
from math import radians, degrees, log10
from enum import Enum, auto
from statistics import mode, StatisticsError
from itertools import combinations
from functools import reduce
from operator import mul as multiply_operator


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
        if face.Surface.TypeId == "Part::GeomCylinder":
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
    try:
        thickness_value = mode(combined_list_of_thicknesses)
        return thickness_value
    except StatisticsError:
        return 0.0


def estimate_thickness_from_face(shape: Part.Shape, selected_face: int) -> float:
    ref_face = shape.Faces[selected_face]
    # find all planar faces that are parallel to the chosen face
    candidates = [
        f
        for f in shape.Faces
        if f.hashCode() != ref_face.hashCode()
        and f.Surface.TypeId == "Part::GeomPlane"
        and ref_face.Surface.Axis.isParallel(f.Surface.Axis, eps_angular)
    ]
    if not candidates:
        return 0.0
    opposite_face = sorted(candidates, key=lambda x: abs(x.Area - ref_face.Area))[0]
    return abs(
        opposite_face.valueAt(0, 0).distanceToPlane(
            ref_face.Surface.Position, ref_face.Surface.Axis
        )
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
    return abs(abs(s.Center.distanceToPlane(p.Position, p.Axis)) - s.Radius) < eps


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
    # weird shape mutability nonsense? -> have to use indexes, because the
    # underlying geometry may get changed around while building the graph
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
    # return None if there is nothing tangent to the seed face
    return None


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
    # the u period is always positive: 0.0 <= umin < umax <= 2*pi
    umin, umax, vmin, vmax = cylindrical_face.ParameterRange
    bend_angle = umax - umin
    radius = cylindrical_face.Surface.Radius
    bend_direction = {"Reversed": "Up", "Forward": "Down"}[cylindrical_face.Orientation]
    if bend_direction == "Up":
        bend_allowance = (radius + k_factor * thickness) * bend_angle
    else:
        bend_allowance = (radius - thickness * (1 - k_factor)) * bend_angle
    overall_height = abs(vmax - vmin)
    y_scale_factor = radius * bend_allowance / (radius * bend_angle)
    wire = Part.Wire()
    for e in cylindrical_face.Edges:
        edge_on_surface, e_param_min, e_param_max = cylindrical_face.curveOnSurface(e)
        if isinstance(edge_on_surface, Part.Geom2d.Line2d):
            v1 = edge_on_surface.value(e_param_min)
            y1, x1 = v1.x - umin, v1.y - vmin
            v2 = edge_on_surface.value(e_param_max)
            y2, x2 = v2.x - umin, v2.y - vmin
            line = Part.makeLine(
                Vector(x1, y1 * y_scale_factor), Vector(x2, y2 * y_scale_factor)
            )
            wire.add(line)
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
        else:
            errmsg = (
                f"Unhandled curve type when unfolding face: {type(edge_on_surface)}"
            )
            raise TypeError(errmsg)
    face = Part.makeFace(wire, "Part::FaceMakerBullseye")
    mirror_base_pos = Vector(overall_height / 2, bend_allowance / 2)
    match refpos:
        case UVRef.BOTTOM_LEFT:
            return face
        case UVRef.BOTTOM_RIGHT:
            return face.mirror(mirror_base_pos, Vector(1, 0))
        case UVRef.TOP_LEFT:
            return face.mirror(mirror_base_pos, Vector(0, 1))
        case UVRef.TOP_RIGHT:
            return face.mirror(mirror_base_pos, Vector(0, 1)).mirror(
                mirror_base_pos, Vector(1, 0)
            )


def compute_unbend_transform(
    bent_face: Part.Face, base_edge: Part.Edge, thickness: float, k_factor: float
) -> tuple:
    # for cylindrical surfaces, the u-parameter corresponds to the radial
    # direction, and the u-period is the radial boundary of the cylindrical
    # patch. The v-period corresponds to the axial direction.
    umin, umax, vmin, vmax = bent_face.ParameterRange
    # the u period is always positive: 0.0 <= umin < umax <= 2*pi
    bend_angle = umax - umin
    radius = bent_face.Surface.Radius
    # disallow fully cylindrical bends. These can't be formed because the
    # opposite edge of the sheet will intersect the previous face
    if bend_angle > radians(359.9):
        errmsg = "Bend angle must be less that 359.9 degrees"
        raise RuntimeError(errmsg)
    # for cylindrical surfaces:
    # the surface of the outside of a tube ('convex') is always forward-oriented
    # the surface of the inside of a tube ('concave') is always reverse-oriented
    bend_direction = {"Reversed": "Up", "Forward": "Down"}[bent_face.Orientation]
    # the reference edge should intersect with the bent cylindrical surface at
    # either opposite corner of surface's uv-parameter range.
    # We need to determine which of these possibilities is correct
    first_corner_point = bent_face.valueAt(umin, vmin)
    second_corner_point = bent_face.valueAt(umax, vmin)
    # at least one of these points should be on the starting edge
    dist1 = first_corner_point.distanceToLine(
        base_edge.Curve.Location, base_edge.Curve.Direction
    )
    dist2 = second_corner_point.distanceToLine(
        base_edge.Curve.Location, base_edge.Curve.Direction
    )
    # the x-axis of our desired reference is the tangent vector to a radial
    # line on the cylindrical surface, oriented away from the previous face.
    # We can compute candidates to choose from with the .tangent() method
    if dist1 < eps:  # 'Forward' orientation
        tangent_vector, binormal_vector = bent_face.Surface.tangent(umin, vmin)
        y_axis = tangent_vector
        # use the normal of the face and not the surface here
        # If the face is reverse oriented, the surface normal will be flipped
        # relative to the face normal.
        z_axis = bent_face.normalAt(umin, vmin)
        # place the reference point such that the cylindrical face lies in the
        # (+x, +y) quadrant of the xy-plane of the reference coordinate system
        x_axis = y_axis.cross(z_axis)
        if x_axis.dot(corner_1 := bent_face.valueAt(umin, vmin)) < x_axis.dot(
            corner_2 := bent_face.valueAt(umin, vmax)
        ):
            lcs_base_point = corner_1
            uvref = UVRef.BOTTOM_LEFT
        else:
            lcs_base_point = corner_2
            uvref = UVRef.TOP_LEFT
    elif dist2 < eps:  # 'Reverse' orientation
        tangent_vector, binormal_vector = bent_face.Surface.tangent(umax, vmin)
        y_axis = tangent_vector.negative()
        z_axis = bent_face.normalAt(umax, vmin)
        x_axis = y_axis.cross(z_axis)
        if x_axis.dot(corner_3 := bent_face.valueAt(umax, vmin)) < x_axis.dot(
            corner_4 := bent_face.valueAt(umax, vmax)
        ):
            lcs_base_point = corner_3
            uvref = UVRef.BOTTOM_RIGHT
        else:
            lcs_base_point = corner_4
            uvref = UVRef.TOP_RIGHT
    else:
        errmsg = "No point on reference edge"
        raise RuntimeError(errmsg)
    # note that the x-axis is ignored here based on the priority string
    lcs_rotation = Rotation(x_axis, y_axis, z_axis, "ZYX")
    alignment_transform = Placement(lcs_base_point, lcs_rotation).toMatrix()
    # the actual unbend transformation is found by reversing the rotation of
    # a flat face after the bend due to the bending operation,
    # then pushing it forward according to the bend allowance
    if bend_direction == "Up":
        bend_allowance = (radius + k_factor * thickness) * bend_angle
    else:
        bend_allowance = (radius - thickness * (1 - k_factor)) * bend_angle
    # fmt: off
    allowance_transform = Matrix(
        1, 0, 0, 0,
        0, 1, 0, bend_allowance,
        0, 0, 1, 0,
        0, 0, 0, 1
    )
    rot = Rotation(
        Vector(1, 0, 0),
        (-1 if bend_direction == "Up" else 1) * degrees(bend_angle)
    ).toMatrix()
    translate = Matrix(
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, (1 if bend_direction == "Up" else -1) * radius,
        0, 0, 0, 1
    )
    # fmt: on
    # compose transformations to get the final matrix
    overall_transform = Matrix()
    overall_transform.transform(Vector(), alignment_transform.inverse())
    overall_transform.transform(Vector(), translate * rot * translate.inverse())
    overall_transform.transform(Vector(), allowance_transform)
    overall_transform.transform(Vector(), alignment_transform)
    return alignment_transform, overall_transform, uvref


def unfold(shape: Part.Shape, root_face_index: int, k_factor: int) -> Part.Shape:
    graph_of_sheet_faces = build_graph_of_tangent_faces(shp, root_face)
    if not graph_of_sheet_faces:
        errmsg = (
            "No faces were found that are tangent to the selected face. "
            "Try selecting a different face, and/"
            "or confirm that the shape is a watertight solid."
        )
        raise RuntimeError(errmsg)
    thickness = estimate_thickness_from_cylinders(shape)
    if not thickness:
        thickness = estimate_thickness_from_face(shape, root_face_index)
    if not thickness:
        errmsg = "Couldn't estimate thickness for shape!"
        raise RuntimeError(errmsg)
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
                "Part::GeomToroid": "purple",
                "Part::GeomCone": "green",
                "Part::GeomSurfaceOfExtrusion": "orange",
                "Part::GeomSphere": "hotpink",
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
    FreeCAD.Console.PrintLog(
        "Network of tangent faces:\n" + str(nx.nx_pydot.to_pydot(dg))
    )
    # the digraph should now have everything we need to unfold the shape
    # unfold bends adjacent to 2 planar faces
    for e in [
        e for e in dg.edges if shp.Faces[e[1]].Surface.TypeId == "Part::GeomCylinder"
    ]:
        bend_part = shp.Faces[e[1]]
        edge_before_bend_index = dg.get_edge_data(e[0], e[1])["label"]
        edge_before_bend = shp.Edges[edge_before_bend_index]
        if edge_before_bend.Curve.TypeId != "Part::GeomLine":
            errmsg = (
                "This shape appears to have bends across non-straight edges. "
                "Unfolding such a shape is not yet supported."
                f" (Edge{edge_before_bend_index + 1})"
            )
            raise RuntimeError(errmsg)
        alignment_transform, overall_transform, uvref = compute_unbend_transform(
            bend_part, edge_before_bend, thickness, k_factor
        )
        unbent_face = unroll_cylinder(bend_part, uvref, k_factor, thickness)
        # add the transformation and unbend shape to the end node of the edge as attributes
        dg.nodes[e[1]]["unbent_shape"] = unbent_face.transformed(alignment_transform)
        dg.nodes[e[1]]["unbend_transform"] = overall_transform
    # get a path from the root (stationary) face to each other face
    # so we can combine transformations to position the final shape
    list_of_faces = []
    for face_id, path in nx.shortest_path(dg, source=root_face_index).items():
        path_to_face = path[
            :-1
        ]  # the path includes the face itself, which we don't need
        ndat = dg.nodes.data()
        list_of_matrices = [
            ndat[f]["unbend_transform"]
            for f in path_to_face
            if "unbend_transform" in ndat[f]
        ]
        final_mat = reduce(multiply_operator, list_of_matrices, Matrix())
        if "unbent_shape" in ndat[face_id]:
            final_face = ndat[face_id]["unbent_shape"]
        else:
            final_face = shape.Faces[face_id]
        list_of_faces.append(final_face.transformed(final_mat))
    extrude_vec = shp.Faces[root_face_index].normalAt(0, 0).normalize() * -1 * thickness
    solid_components = [f.extrude(extrude_vec) for f in list_of_faces]
    # TODO: optimize use of fuzzy boolean options
    fuzzy_tolerance = eps * 1000
    solid = solid_components[0].multiFuse(solid_components[1:], fuzzy_tolerance)
    return solid.removeSplitter()


if __name__ == "__main__":
    selection_object = FreeCAD.Gui.Selection.getCompleteSelection()[0]
    shp = selection_object.Object.Shape
    root_face = int(selection_object.SubElementNames[0][4:]) - 1
    unfolded_shape = unfold(shp, root_face, k_factor=0.5)
    Part.show(unfolded_shape, selection_object.Object.Label + "_Unfold")
