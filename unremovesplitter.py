import FreeCAD
import FreeCADGui
import Part
from itertools import combinations
from functools import lru_cache
from math import log10
from statistics import mode
from hashlib import md5

eps = FreeCAD.Base.Precision.approximation()


def round_vector(vec: FreeCAD.Vector, ndigits: int = None) -> FreeCAD.Vector:
    return FreeCAD.Vector(*[round(d, ndigits) for d in vec])


def estimate_thickness_from_edges(shp: Part.Shape) -> float:
    # assumes that the modal edge length equals the thickness of the sheetmetal part
    num_places = abs(int(log10(eps)))
    elist = [e.Length for e in shp.Edges]
    elist_rounded = [round(length, num_places) for length in elist]
    return mode(elist_rounded)


def estimate_thickness_from_deps(docobj: FreeCAD.DocumentObject) -> float:
    # find a basebend object that this depends on and use it's thickness
    return 1.0


def estimate_thickness_from_subshape(shp: Part.Shape, subshape: Part.Shape) -> float:
    # estimate thickness using a raycast across a selected face
    return 1.0


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
    # print(curv_map)
    combined_list_of_thicknesses = []
    for radset in curv_map.values():
        if len(radset) > 1:
            for r1, r2 in combinations(radset, 2):
                if (val := abs(r1 - r2)) > eps:
                    combined_list_of_thicknesses.append(val)
    return mode(combined_list_of_thicknesses)


@lru_cache(maxsize=1024)
def straight_edge_hash(e: Part.Edge) -> str:
    hashed = md5()
    vertexes = [*e.Vertexes[0].Point, *e.Vertexes[1].Point]
    for v in vertexes:
        hashed.update(v.hex().encode())
    return hashed.digest()


def unRemoveSplitter(shp: Part.Shape) -> Part.Shape:
    sheet_thickness = estimate_thickness_from_cylinders(shp)
    list_of_faces = []
    for face in shp.Faces:
        if isinstance(face.Surface, Part.Plane):
            # add splitters as needed
            if len(face.Edges) > 4:  # and doesnot_have_holes(face):
                # the thickness of the sheet should be the length of the shortest straight edge on this face
                # sheet_thickness = min(sorted([e.Length for e in face.Edges if isinstance(e.Curve, Part.Line)]))
                existing_straight_edges = [
                    straight_edge_hash(e)
                    for e in face.Edges
                    if isinstance(e.Curve, Part.Line)
                ]
                new_edges = []
                for v1, v2 in combinations(face.Vertexes, 2):
                    if abs(v1.Point.distanceToPoint(v2.Point) - sheet_thickness) < eps:
                        # don't add identical existing edges
                        new_e = Part.makeLine(v1.Point, v2.Point)
                        if straight_edge_hash(new_e) not in existing_straight_edges:
                            new_edges.append(new_e)
                # slice up the face with the edge list
                compound = Part.makeCompound(new_edges)
                # face is planar, normal is the same at any UV coord
                face_normal = face.Surface.normal(0, 0).normalize()
                cutter = compound.extrude(face_normal).translated(-0.5 * face_normal)
                # Part.show(cutter)
                # Part.show(face)
                list_of_faces.extend(face.cut(cutter).Faces)
            else:
                list_of_faces.append(face)
        else:
            list_of_faces.append(face)
    # return Part.Shape()
    return Part.makeSolid(Part.Shell(list_of_faces))


def removeSplitterPoints(shp: Part.Shape) -> None:
    # we are targeting circular and/or straight edges that have at least one vertex that is coincident to only 2 edges in the overall shape
    new_faces = []
    for face in shp.Faces:
        circular_edges = [e for e in face.Edges if isinstance(e.Curve, Part.Circle)]
        if circular_edges and (
            isinstance(face.Surface, Part.Plane)
            or isinstance(face.Surface, Part.Cylinder)
        ):
            new_edges = [e for e in face.Edges]
            for c1, c2 in combinations(circular_edges, 2):
                if (
                    c1.Curve.Axis.isParallel(c2.Curve.Axis, eps)
                    and c1.Curve.Center.distanceToPoint(c1.Curve.Center) < eps
                    and abs(c1.Curve.Radius - c2.Curve.Radius) < eps
                ):
                    # add in the new combined edge. brute force the 4 possible cases,
                    # rather than relying on edges to be oriented in the same direction
                    if c1.Vertexes[0].Point.distanceToPoint(c2.Vertexes[0].Point) < eps:
                        p1 = c1.Vertexes[1].Point
                        p2 = c1.Vertexes[0].Point
                        p3 = c2.Vertexes[1].Point
                    elif (
                        c1.Vertexes[1].Point.distanceToPoint(c2.Vertexes[0].Point) < eps
                    ):
                        p1 = c1.Vertexes[0].Point
                        p2 = c1.Vertexes[1].Point
                        p3 = c2.Vertexes[1].Point
                    elif (
                        c1.Vertexes[1].Point.distanceToPoint(c2.Vertexes[1].Point) < eps
                    ):
                        p1 = c1.Vertexes[0].Point
                        p2 = c1.Vertexes[1].Point
                        p3 = c2.Vertexes[0].Point
                    elif (
                        c1.Vertexes[0].Point.distanceToPoint(c2.Vertexes[1].Point) < eps
                    ):
                        p1 = c1.Vertexes[1].Point
                        p2 = c1.Vertexes[0].Point
                        p3 = c2.Vertexes[0].Point
                    else:
                        # coaxial edges that don't touch
                        continue
                    # TODO: add a continue case if the shared vertex p2 is
                    # coincident to more than 2 edges in the global shape

                    # pop the existing edges out of the final list if needed
                    to_pop = []
                    for i, e in enumerate(new_edges):
                        if c1.isSame(e) or c2.isSame(e):
                            to_pop.append(i)
                    for j in sorted(to_pop, reverse=True):
                        new_edges.pop(j)
                    # add the combined edge to the final shape
                    new_circular_edge = Part.Arc(p1, p2, p3)
                    new_edges.append(new_circular_edge.toShape())
            # add the adjusted face back into the shape
            if len(new_edges) != len(face.Edges):
                if isinstance(face.Surface, Part.Plane):
                    # face is planar
                    # print(new_edges)
                    new_faces.append(
                        Part.makeFace(
                            Part.Wire(Part.sortEdges(new_edges)[0]),
                            "Part::FaceMakerBullseye",
                        )
                    )

                else:  # face is cylindrical
                    surface_to_cut = face.Surface
                    wire = Part.Wire(Part.sortEdges(new_edges)[0])
                    # what the fuck
                    # Part.show(surface_to_cut.toShape().generalFuse(wire)[0], "genfuse")
                    res = sorted(
                        surface_to_cut.toShape().generalFuse(wire)[0].Faces,
                        key=lambda f: abs(f.Area - face.Area),
                    )[0]
                    # Part.show(Part.makeCompound([surface_to_cut, wire]), f"CompoundToCut_{len(new_edges)}_{len(face.Edges)}")
                    # Part.show(res)
                    new_faces.append(res)
            else:
                new_faces.append(face)
        else:
            new_faces.append(face)

    return Part.makeSolid(Part.Shell(new_faces))


if __name__ == "__main__":
    sel = FreeCADGui.Selection.getSelection()[0]
    Part.show(unRemoveSplitter(sel.Shape))
    # Part.show(unRemoveSplitter(removeSplitterPoints(sel.Shape)))
    # Part.show(removeSplitterPoints(sel.Shape))
    # print(f"Estimated sheet metal thickness: {estimate_thickness_from_cylinders(sel.Shape)}")
