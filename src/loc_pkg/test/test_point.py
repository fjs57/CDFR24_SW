import sys
sys.path.append("../")
from loc_pkg.localization.Point import Point
import numpy as np

import unittest

class TestPoint(unittest.TestCase):
    def test_compute_cartesian(self):
        p = Point()
        
        p.pos_r = 0
        p.pos_th = 0
        p.compute_cartesian()
        self.assertAlmostEqual(0, p.pos_x, delta=1e-6)
        self.assertAlmostEqual(0, p.pos_y, delta=1e-6)

        p.pos_r = 1
        p.pos_th = 0
        p.compute_cartesian()
        self.assertAlmostEqual(1.0, p.pos_x, delta=1e-6)
        self.assertAlmostEqual(0.0, p.pos_y, delta=1e-6)

        p.pos_r = 1
        p.pos_th = np.pi/2
        p.compute_cartesian()
        self.assertAlmostEqual(0.0, p.pos_x, delta=1e-6)
        self.assertAlmostEqual(1.0, p.pos_y, delta=1e-6)

    def test_compute_polar(self):
        p=Point()

        p.pos_x = 5
        p.pos_y = 0
        p.compute_polar()
        self.assertAlmostEqual(5.0, p.pos_r, delta=1e-6)
        self.assertAlmostEqual(0.0, p.pos_th, delta=1e-6)

        p.pos_x = 0
        p.pos_y = 5
        p.compute_polar()
        self.assertAlmostEqual(5.0, p.pos_r, delta=1e-6)
        self.assertAlmostEqual(np.pi/2, p.pos_th, delta=1e-6)

        p.pos_x = 0
        p.pos_y = -5
        p.compute_polar()
        self.assertAlmostEqual(5.0, p.pos_r, delta=1e-6)
        self.assertAlmostEqual(-np.pi/2, p.pos_th, delta=1e-6)

        p.pos_x = -5
        p.pos_y = 0
        p.compute_polar()
        self.assertAlmostEqual(5.0, p.pos_r, delta=1e-6)
        self.assertAlmostEqual(np.pi, p.pos_th, delta=1e-6)

    def test_get_cartesian_list(self):
        p=Point()
        p.pos_x = 123.0
        p.pos_y = 456.0
        self.assertEqual([123.0, 456.0], p.get_cartesian_list())

    def test_angle_distance_between_points(self):
        a = Point()
        b = Point()
        a.pos_r = 1.0
        a.pos_th = 0.0
        b.pos_r = 1.0
        b.pos_th = np.pi/2

        du, au = a.get_distance_and_angle_from_reference(b)
        dv, av = a.get_distance_and_angle_to_point(b)
        self.assertAlmostEqual(du, np.sqrt(2), delta=1e-6)
        self.assertAlmostEqual(dv, np.sqrt(2), delta=1e-6)
        self.assertAlmostEqual(au, -np.pi/4, delta=1e-6)
        self.assertAlmostEqual(av, 3*np.pi/4, delta=1e-6)

        du, au = b.get_distance_and_angle_from_reference(a)
        dv, av = b.get_distance_and_angle_to_point(a)
        self.assertAlmostEqual(du, np.sqrt(2), delta=1e-6)
        self.assertAlmostEqual(dv, np.sqrt(2), delta=1e-6)
        self.assertAlmostEqual(au, 3*np.pi/4, delta=1e-6)
        self.assertAlmostEqual(av, -np.pi/4, delta=1e-6)

    def test_transltion(self):
        p = Point()
        p.pos_r = 1.0
        p.pos_th = np.pi/2
        p.translate(1.0, -1.0)
        self.assertAlmostEqual(p.pos_x, 1.0, delta=1e-6)
        self.assertAlmostEqual(p.pos_y, 0.0, delta=1e-6)

    def test_rotation(self):
        p = Point()
        p.pos_x = 0.0
        p.pos_y = 1.0
        p.rotate(-np.pi/2)
        self.assertAlmostEqual(p.pos_r, 1.0, delta=1e-6)
        self.assertAlmostEqual(p.pos_th, 0.0, delta=1e-6)



if __name__ == "__main__":
    unittest.main()