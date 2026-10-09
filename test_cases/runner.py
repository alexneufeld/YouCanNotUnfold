import os
from dataclasses import dataclass

import FreeCAD
import Import
import importDXF
import Part
import SheetMetalNewUnfolder as uf


@dataclass
class TestCase:
    relpath: str
    face_number: int
    unfolds: bool


test_data = (
    TestCase(*x)
    for x in [
        ("advanced_features/001_bridge_lance.step", 0, False),
        ("advanced_features/002_formed_gusset.step", 0, False),
        ("advanced_features/003_swept_flange.step", 0, False),
        ("advanced_features/004_open_dimple.step", 0, False),
        ("advanced_features/005_louvre.step", 0, False),
        ("advanced_features/006_closed_round_dimple.step", 0, False),
        ("advanced_features/007_embossed_and_bent.step", 0, False),
        ("advanced_features/008_embossed_and_bent_fullradius.step", 0, False),
        ("advanced_features/009_conical_bend.step", 0, False),
        ("advanced_features/010_conic_and_joggle.step", 0, False),
        ("advanced_features/011_coiled_surface_of_extrusion.step", 0, False),
        ("advanced_features/012_staked_corner.step", 0, True),
        ("advanced_features/013_alternating_double_bend.step", 19, True),
        ("complex_objects/001_lots_of_simple_bends.step", 67, True),
        ("complex_objects/002_bend_up_to_face.step", 13, True),
        ("complex_objects/003_sheetmetal_issue_6.step", 36, True),
        ("complex_objects/004_sheetmetal_issue_203.step", 4, True),
        ("complex_objects/005_sheetmetal_issue_254.step", 11, True),
        ("complex_objects/006_sheetmetal_issue_274.step", 17, True),
        ("complex_objects/007_sheetmetal_issue_269.step", 3, True),
        ("complex_objects/008_sheetmetal_issue_29.step", 18, False),
        ("complex_objects/009_sheetmetal_issue_18.step", 17, True),
        ("complex_objects/010_sheetmetal_issue_11.step", 18, True),
        ("complex_objects/011_freecad_forum_t=27629#p222854.step", 30, True),
        ("complex_objects/012_freecad_forum_t=27629&start=20#p223601.step", 30, True),
        ("complex_objects/013_freecad_forum_t=17706.step", 27, True),
        ("complex_objects/014_freecad_forum_t=60818&start=50#p145472.step", 14, True),
        ("complex_objects/015_freecad_forum_t=60818&start=70#p270793.step", 27, True),
        ("complex_objects/016_freecad_forum_t=60818&start=120#p382549.step", 96, True),
        ("complex_objects/017_freecad_forum_t=60818&start=130#p391563.step", 63, True),
        ("complex_objects/018_freecad_forum_t=60818&start=150#p404021.step", 46, True),
        ("complex_objects/019_freecad_forum_t=60818&start=160#p404343.step", 69, True),
        ("complex_objects/020_freecad_forum_t=60818&start=160#p404351.step", 36, True),
        ("complex_objects/021_freecad_forum_t=60818&start=200#p437761.step", 4, True),
        ("complex_objects/022_freecad_forum_t=60818&start=230#p445390.step", 14, True),
        ("complex_objects/023_freecad_forum_t=60818&start=250#p453455.step", 4, True),
        ("complex_objects/024_freecad_forum_t=60818&start=290#p468078.step", 47, True),
        ("complex_objects/025_array_of_holes.step", 17, True),
        ("fcstd_files/double_bend.FCStd", 0, True),
        ("fcstd_files/fails_bopcheck.FCStd", 0, True),
        ("fcstd_files/fold_with_holes.FCStd", 0, True),
        ("fcstd_files/k_factor.FCStd", 0, True),
        ("fcstd_files/material_definition_sheets.FCStd", 0, True),
        ("fcstd_files/sheetmetal_issue_345.FCStd", 0, True),
        ("fcstd_files/tangent_faces.FCStd", 0, True),
        ("simple_features/001_flat_plate.step", 5, True),
        ("simple_features/002_single_bend.step", 3, True),
        ("simple_features/003_single_hem.step", 1, True),
        ("simple_features/004_joggle_bend.step", 8, True),
        ("simple_features/005_corner_bend.step", 1, True),
        ("simple_features/006_corner_bend_with_relief.step", 1, True),
        ("simple_features/007_single_bend_refined.step", 1, True),
        ("simple_features/008_box_with_closed_corners.step", 6, True),
    ]
)


def dxf_to_face(fpath):
    doc = FreeCAD.newDocument(hidden=True, temp=True)
    importDXF.insert(fpath, doc.Name)
    edges = []
    for layer in [
        x for x in doc.Objects if hasattr(x, "Proxy") and x.Proxy.Type == "Layer"
    ]:
        if (
            layer.Label.endswith("_Sketch")
            or layer.Label.endswith("_Sketch_Holes")
            or layer.Label.endswith("_Sketch_Internal")
        ):
            edges.extend(list(x.Shape.Edges[0] for x in layer.Group))

    wirelist = uf.Edge2DCleanup.clean_and_structure_geometry(edges)
    face = Part.makeFace(wirelist, "Part::FaceMakerCheese")
    FreeCAD.closeDocument(doc.Name)
    return face


def run_unfolder_on_stepfile(fpath, faceindex, bac):
    doc = FreeCAD.newDocument(hidden=True, temp=True)
    feature = Import.insert(fpath, doc.Name)[0][0]
    rootface, unbend, trimmed_bend_lines, root_normal, bend_lines_info = uf.getUnfold(
        bac, feature, f"Face{faceindex+1}"
    )
    # print("unfold successful!")
    FreeCAD.closeDocument(doc.Name)
    return unbend


def get_stepfile(fpath):
    shp = Part.Shape()
    shp.read(fpath)
    return shp


if __name__ == "__main__":
    root_path = "/home/alex/Code/YouCanNotUnfold/test_cases/"
    dxf_path = os.path.join(root_path, "dxf")
    bac = uf.BendAllowanceCalculator.from_single_value(
        0.4, uf.BendAllowanceCalculator.KFactorStandard.ANSI
    )
    doc = FreeCAD.ActiveDocument

    for testcase in test_data:
        testcase_dir = os.path.dirname(testcase.relpath)
        if not testcase.unfolds:
            print(f"Skipping test case: {testcase.relpath}")
            continue
        if testcase.relpath.endswith(".FCStd"):
            print(f"Skipping FreeCAD file: {testcase.relpath}")
            continue
        unbend_shape = run_unfolder_on_stepfile(
            os.path.join(root_path, testcase.relpath), testcase.face_number, bac
        )
        canonical_unbend_shape = get_stepfile(
            os.path.join(root_path, "unfolded", testcase.relpath)
        )
        boolresult = unbend_shape.cut(canonical_unbend_shape, 0.05)
        if boolresult.Volume == 0.0:
            print(f"{testcase.relpath}: success!")
        else:
            print(f"{testcase.relpath}: fail!")
            # FreeCAD.setActiveDocument(doc.Name)
            Part.show(boolresult)
        FreeCAD.Gui.updateGui()
