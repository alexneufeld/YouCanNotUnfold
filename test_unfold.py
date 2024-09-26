import os
import sys

if "FREECADPATH" in os.environ:
    sys.path.append(os.environ["FREECADPATH"])
else:
    raise RuntimeError("Please specify the FREECADPATH environment variable")

from math import sqrt
from unittest import TestCase

import FreeCAD
import Part
from FreeCAD import Vector

import unfold

# used when comparing positions in 3D space
eps = FreeCAD.Base.Precision.approximation()
# used when comparing angles
eps_angular = FreeCAD.Base.Precision.angular()


class TestIsFacesTangent(TestCase):
    def test_plane_plane(self):
        p1 = Part.Plane(1, 2, 3, 4).toShape()
        p2 = Part.Plane(p1.Surface, eps / 10).toShape()
        p3 = Part.Plane(4, 3, 2, 1).toShape()

        # different planes aren't tangent
        self.assertFalse(unfold.is_faces_tangent(p1, p3))

        # planes offset by less than FreeCAD's equality tolerance are still
        # considered tangent
        self.assertTrue(unfold.is_faces_tangent(p1, p2))

    def test_plane_cylinder(self):
        p1 = Part.Plane(Vector(0, 0, 0), Vector(1, 0, 1)).toShape()
        p2 = Part.Plane(Vector(1, 0, 0), Vector(1, 0, 0)).toShape()
        c1 = Part.Cylinder().toShape()

        self.assertTrue(unfold.is_faces_tangent(p2, c1))
        self.assertFalse(unfold.is_faces_tangent(p1, c1))

    def test_cylinder_cylinder(self):
        cylinder_1 = Part.makeCylinder(1, 10, Vector(0, 0, 0), Vector(1, 0, 0))
        c1 = [f for f in cylinder_1.Faces if f.Surface.TypeId == "Part::GeomCylinder"][
            0
        ]
        cylinder_2 = Part.makeCylinder(2, 10, Vector(0, 0, 3), Vector(1, 0, 0))
        c2 = [f for f in cylinder_2.Faces if f.Surface.TypeId == "Part::GeomCylinder"][
            0
        ]
        cylinder_3 = Part.makeCylinder(2, 10, Vector(0, 0, 3), Vector(0, 1, 0))
        c3 = [f for f in cylinder_3.Faces if f.Surface.TypeId == "Part::GeomCylinder"][
            0
        ]

        self.assertTrue(unfold.is_faces_tangent(c1, c2))
        self.assertFalse(unfold.is_faces_tangent(c1, c3))

    def test_plane_torus(self):
        t1 = Part.makeTorus(10, 2, Vector(0, 0, 2), Vector(0, 0, 1)).Faces[0]
        p1 = Part.Plane().toShape()
        p2 = Part.Plane(Vector(0, 0, 0), Vector(1, 0, 0)).toShape()

        self.assertTrue(unfold.is_faces_tangent(t1, p1))
        self.assertFalse(unfold.is_faces_tangent(t1, p2))

    def test_cylinder_torus(self):
        t1 = Part.makeTorus(10, 2, Vector(0, 0, 2), Vector(0, 0, 1)).Faces[0]
        cylinder_1 = Part.makeCylinder(8, 10, Vector(0, 0, 0), Vector(0, 0, 1))
        c1 = [f for f in cylinder_1.Faces if f.Surface.TypeId == "Part::GeomCylinder"][
            0
        ]
        cylinder_2 = Part.makeCylinder(2, 10, Vector(0, 10, 2), Vector(1, 0, 0))
        c2 = [f for f in cylinder_2.Faces if f.Surface.TypeId == "Part::GeomCylinder"][
            0
        ]
        cylinder_3 = Part.makeCylinder(2, 10, Vector(0, 0, 0), Vector(1, 1, 1))
        c3 = [f for f in cylinder_3.Faces if f.Surface.TypeId == "Part::GeomCylinder"][
            0
        ]
        cylinder_4 = Part.makeCylinder(12, 10, Vector(0, 0, 0), Vector(0, 0, 1))
        c4 = [f for f in cylinder_4.Faces if f.Surface.TypeId == "Part::GeomCylinder"][
            0
        ]

        self.assertTrue(unfold.is_faces_tangent(t1, c1))
        self.assertTrue(unfold.is_faces_tangent(t1, c2))
        self.assertTrue(unfold.is_faces_tangent(t1, c4))
        self.assertFalse(unfold.is_faces_tangent(t1, c3))

    def test_sphere_sphere(self):
        s1 = Part.makeSphere(10, Vector(1, 2, 3)).Faces[0]
        s2 = Part.makeSphere(10 + 0.1 * eps, Vector(1, 2, 3)).Faces[0]
        s3 = Part.makeSphere(10, Vector(3, 2, 1)).Faces[0]

        self.assertTrue(unfold.is_faces_tangent(s1, s2))
        self.assertFalse(unfold.is_faces_tangent(s1, s3))


class TestUnrollCylinder(TestCase):
    def test_unroll_simple_face(self):
        arc = Part.Arc(
            Vector(0, 0, 0),
            Vector(sqrt(2) / 2 * 10, 0, 10 - sqrt(2) / 2 * 10),
            Vector(10, 0, 10),
        ).toShape()
        face = arc.extrude(Vector(0, 10, 0))
        self.assertTrue(face is not None)
