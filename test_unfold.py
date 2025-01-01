import os
import sys

if "FREECADPATH" in os.environ:
    sys.path.append(os.environ["FREECADPATH"])
else:
    raise RuntimeError("Please specify the FREECADPATH environment variable")

from unittest import TestCase

import FreeCAD

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
