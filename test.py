import unittest
from path_tracer import Vec3, Triangle, Sphere, Ray

class IntersectionTest(unittest.TestCase):
    def test_triangle(self):
        t = Triangle(Vec3(0.0, 0.0, 0.0),
                     Vec3(1.0, 0.0, 0.0),
                     Vec3(0.0, 1.0, 0.0))
        no_intersecting_rays = [
            Ray(Vec3(0, 0, 1), Vec3(1, 0, 0)),
            Ray(Vec3(0, 0, 1), Vec3(-1, 0, 0)),
            Ray(Vec3(1, 1, -1), Vec3(-1, 0, 0)),
            Ray(Vec3(-1, -1, -1), Vec3(0, 1, 0)),
        ]
        
        for r in no_intersecting_rays:
            self.assertEqual(r.intersect(t).success, False)

        intersecting_rays = [
            Ray(Vec3(0.1, 0.1, -1), Vec3(0, 0, 1)),
            Ray(Vec3(0.5, 0.5, 1), Vec3(0, 0, -1)),
            Ray(Vec3(0.5, 0.5, -1), Vec3(0, 0, 1)),
            Ray(Vec3(0.5, 0.5, 1), Vec3(0, 0, -1)),
        ]

        for r in intersecting_rays:
            self.assertEqual(r.intersect(t).success, True)

    def test_sphere(self):
        s = Sphere(Vec3.zero, 1.0)

        no_intersecting_rays = [
            Ray(Vec3(0, 0, 2), Vec3(1, 0, 0)),
            Ray(Vec3(0, 0, 2), Vec3(-1, 0, 0)),
            Ray(Vec3(1, 1, -2), Vec3(-1, 0, 0)),
            Ray(Vec3(-2, -1, -1), Vec3(0, 1, 0)),
        ]
        
        for r in no_intersecting_rays:
            self.assertEqual(r.intersect(s).success, False)

        intersecting_rays = [
            Ray(Vec3(0, 0, -2), Vec3(0, 0, 1)),
            Ray(Vec3(0.5, -2, 0.5), Vec3(0, 1, 0)),
            Ray(Vec3(-2, 0.5, 0), Vec3(1, 0, 0)),
            Ray(Vec3(0.5, 0.5, 2), Vec3(0, 0, -1)),
        ]

        for r in intersecting_rays:
            self.assertEqual(r.intersect(s).success, True)

if __name__ == "__main__":
    unittest.main()
