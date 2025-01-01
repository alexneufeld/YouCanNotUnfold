import os
import sys

if "FREECADPATH" in os.environ:
    sys.path.append(os.environ["FREECADPATH"])
else:
    raise RuntimeError("Please specify the FREECADPATH environment variable")

from unittest import TestCase, skip

import FreeCAD
import Part

import unfold

# used when comparing positions in 3D space
eps = FreeCAD.Base.Precision.approximation()
# used when comparing angles
eps_angular = FreeCAD.Base.Precision.angular()

TEST_FILE_DIR = os.path.join(os.path.dirname(__file__), "test_cases")


class TestTangentFaces(TestCase):
    def setUp(self):
        self.doc = FreeCAD.openDocument(
            os.path.join(TEST_FILE_DIR, "fcstd_files", "tangent_faces.FCStd")
        )
        self.p1 = self.doc.Plane.Shape.Faces[0]
        self.p2 = self.doc.Plane001.Shape.Faces[0]
        self.p3 = self.doc.Plane002.Shape.Faces[0]
        self.c1 = self.doc.Cylinder.Shape.Faces[0]
        self.c2 = self.doc.Cylinder001.Shape.Faces[0]
        self.c3 = self.doc.Cylinder002.Shape.Faces[0]
        self.c4 = self.doc.Cylinder003.Shape.Faces[0]
        self.c5 = self.doc.Cylinder004.Shape.Faces[0]
        self.c6 = self.doc.Cylinder005.Shape.Faces[0]
        self.c7 = self.doc.Cylinder006.Shape.Faces[0]
        self.t1 = self.doc.Torus.Shape.Faces[0]
        self.t2 = self.doc.Torus001.Shape.Faces[0]
        self.t3 = self.doc.Torus002.Shape.Faces[0]
        self.t4 = self.doc.Torus003.Shape.Faces[0]
        self.s1 = self.doc.Sphere.Shape.Faces[0]
        self.s2 = self.doc.Sphere001.Shape.Faces[0]
        self.s3 = self.doc.Sphere002.Shape.Faces[0]
        self.s4 = self.doc.Sphere003.Shape.Faces[0]
        self.s5 = self.doc.Sphere004.Shape.Faces[0]
        self.cn1 = self.doc.Cone.Shape.Faces[0]
        self.cn2 = self.doc.Cone001.Shape.Faces[0]
        self.cn3 = self.doc.Cone002.Shape.Faces[0]
        self.cn4 = self.doc.Cone003.Shape.Faces[0]

    def test_plane_plane(self):
        self.assertTrue(unfold.TangentFaces.compare(self.p1, self.p2))
        self.assertFalse(unfold.TangentFaces.compare(self.p2, self.p3))

    def test_plane_cylinder(self):
        self.assertTrue(unfold.TangentFaces.compare(self.p1, self.c2))
        self.assertFalse(unfold.TangentFaces.compare(self.p1, self.c1))

    def test_plane_torus(self):
        self.assertTrue(unfold.TangentFaces.compare(self.p1, self.t1))
        self.assertFalse(unfold.TangentFaces.compare(self.p3, self.t1))

    def test_plane_sphere(self):
        self.assertTrue(unfold.TangentFaces.compare(self.p1, self.s1))
        self.assertFalse(unfold.TangentFaces.compare(self.p1, self.s2))

    def test_plane_cone(self):
        self.assertTrue(unfold.TangentFaces.compare(self.p1, self.cn3))
        self.assertTrue(unfold.TangentFaces.compare(self.p1, self.cn2))
        self.assertFalse(unfold.TangentFaces.compare(self.cn1, self.p1))

    def test_cylinder_cylinder(self):
        self.assertTrue(unfold.TangentFaces.compare(self.c2, self.c3))
        self.assertFalse(unfold.TangentFaces.compare(self.c1, self.c2))

    def test_cylinder_torus(self):
        self.assertTrue(unfold.TangentFaces.compare(self.c4, self.t1))
        self.assertTrue(unfold.TangentFaces.compare(self.c5, self.t1))
        self.assertTrue(unfold.TangentFaces.compare(self.c6, self.t1))
        self.assertFalse(unfold.TangentFaces.compare(self.c1, self.t1))

    def test_cylinder_sphere(self):
        self.assertTrue(unfold.TangentFaces.compare(self.s4, self.c1))
        self.assertTrue(unfold.TangentFaces.compare(self.s5, self.c1))
        self.assertFalse(unfold.TangentFaces.compare(self.s1, self.c1))

    def test_cylinder_cone(self):
        self.assertTrue(unfold.TangentFaces.compare(self.c7, self.cn2))
        self.assertTrue(unfold.TangentFaces.compare(self.c7, self.cn3))
        self.assertTrue(unfold.TangentFaces.compare(self.c7, self.cn3))
        self.assertFalse(unfold.TangentFaces.compare(self.c1, self.cn2))

    def test_torus_torus(self):
        self.assertTrue(unfold.TangentFaces.compare(self.t1, self.t2))
        self.assertTrue(unfold.TangentFaces.compare(self.t1, self.t3))
        self.assertFalse(unfold.TangentFaces.compare(self.t3, self.t4))

    def test_torus_sphere(self):
        pass

    def test_torus_cone(self):
        pass

    def test_sphere_sphere(self):
        pass

    def test_sphere_cone(self):
        pass

    def test_cone_cone(self):
        pass


class TestBendAllowanceCalculator(TestCase):
    def setUp(self):
        test_file = os.path.join(
            TEST_FILE_DIR, "fcstd_files", "material_definition_sheets.FCStd"
        )
        self.doc = FreeCAD.openDocument(test_file)

    def test_from_spreadsheet(self):
        sheet = self.doc.Spreadsheet
        allowance_calculator = unfold.BendAllowanceCalculator.from_spreadsheet(sheet)
        self.assertEqual(allowance_calculator.radius_thickness_values, [1.0, 3.0, 99.0])
        self.assertEqual(allowance_calculator.k_factor_values, [0.38, 0.43, 0.50])
        self.assertEqual(allowance_calculator.get_k_factor(2.0, 2.0), 0.38)
        self.assertEqual(allowance_calculator.get_k_factor(2.0, 99.0), 0.38)
        self.assertEqual(allowance_calculator.get_k_factor(999.0, 1.0), 0.50)
        self.assertAlmostEqual(
            allowance_calculator.get_k_factor(2.0, 1.0), 0.38 + 0.5 * (0.43 - 0.38)
        )

    def test_constant_value(self):
        allowance_calculator = unfold.BendAllowanceCalculator.from_single_value(0.50)
        self.assertEqual(allowance_calculator.get_k_factor(1.0, 999.0), 0.50)
        self.assertEqual(allowance_calculator.get_k_factor(999.0, 1.0), 0.50)


class TestSimpleObjectUnfolding(TestCase):
    def setUp(self):
        pass

    @skip("improve error when nothing needs to be unfolded?")
    def test_flat_plate(self):
        test_file = os.path.join(
            TEST_FILE_DIR, "simple_features", "001_flat_plate.step"
        )
        shp = Part.Shape()
        shp.read(test_file)
        unfolded_solid, bend_lines = unfold.unfold(shp, 5, 0.5)
        self.assertFalse(unfolded_solid.isNull())
        # a simple flat plate has no bend lines when 'unfolded'
        self.assertTrue(bend_lines.isNull())

    def test_single_bend(self):
        test_file = os.path.join(
            TEST_FILE_DIR, "simple_features", "002_single_bend.step"
        )
        shp = Part.Shape()
        shp.read(test_file)
        unfolded_solid, bend_lines = unfold.unfold(shp, 3, 0.5)
        self.assertFalse(unfolded_solid.isNull())
        self.assertFalse(bend_lines.isNull())

    def test_single_hem(self):
        test_file = os.path.join(
            TEST_FILE_DIR, "simple_features", "003_single_hem.step"
        )
        shp = Part.Shape()
        shp.read(test_file)
        unfolded_solid, bend_lines = unfold.unfold(shp, 1, 0.5)
        self.assertFalse(unfolded_solid.isNull())
        self.assertFalse(bend_lines.isNull())

    def test_joggle_bend(self):
        test_file = os.path.join(
            TEST_FILE_DIR, "simple_features", "004_joggle_bend.step"
        )
        shp = Part.Shape()
        shp.read(test_file)
        unfolded_solid, bend_lines = unfold.unfold(shp, 8, 0.5)
        self.assertFalse(unfolded_solid.isNull())
        self.assertFalse(bend_lines.isNull())

    def test_corner_bend(self):
        test_file = os.path.join(
            TEST_FILE_DIR, "simple_features", "005_corner_bend.step"
        )
        shp = Part.Shape()
        shp.read(test_file)
        unfolded_solid, bend_lines = unfold.unfold(shp, 1, 0.5)
        self.assertFalse(unfolded_solid.isNull())
        self.assertFalse(bend_lines.isNull())

    def test_corner_bend_with_relief(self):
        test_file = os.path.join(
            TEST_FILE_DIR, "simple_features", "006_corner_bend_with_relief.step"
        )
        shp = Part.Shape()
        shp.read(test_file)
        unfolded_solid, bend_lines = unfold.unfold(shp, 1, 0.5)
        self.assertFalse(unfolded_solid.isNull())
        self.assertFalse(bend_lines.isNull())

    def test_single_bend_refined(self):
        test_file = os.path.join(
            TEST_FILE_DIR, "simple_features", "007_single_bend_refined.step"
        )
        shp = Part.Shape()
        shp.read(test_file)
        unfolded_solid, bend_lines = unfold.unfold(shp, 1, 0.5)
        self.assertFalse(unfolded_solid.isNull())
        self.assertFalse(bend_lines.isNull())

    def test_box_with_closed_corners(self):
        test_file = os.path.join(
            TEST_FILE_DIR, "simple_features", "007_single_bend_refined.step"
        )
        shp = Part.Shape()
        shp.read(test_file)
        unfolded_solid, bend_lines = unfold.unfold(shp, 6, 0.5)
        self.assertFalse(unfolded_solid.isNull())
        self.assertFalse(bend_lines.isNull())
