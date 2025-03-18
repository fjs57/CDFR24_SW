import sys
sys.path.append("../")
import loc_pkg.localization.loc_utils as loc_utils

import unittest, numpy as np



class TestUtils(unittest.TestCase):
    def test_circle_intersections_no_solutions(self):
        result = loc_utils.cicle_intersections(-1.0, 0.0, 0.5, 1.0, 0.0, 0.5)
        self.assertEqual(result, None)
    
    def test_circle_intersections_one_solution(self):
        result = loc_utils.cicle_intersections(-1.0, 0.0, 1.0, 1.0, 0.0, 1.0)
        self.assertAlmostEqual(result[0][0], 0.0, delta=1e-6)
        self.assertAlmostEqual(result[0][1], 0.0, delta=1e-6)
        self.assertAlmostEqual(result[1][0], 0.0, delta=1e-6)
        self.assertAlmostEqual(result[1][1], 0.0, delta=1e-6)

    def test_circle_intersections_two_solution(self):
        result = loc_utils.cicle_intersections(300, 400, 150, 250, 250, 150)
        self.assertAlmostEqual(result[0][0], 154.066, delta=1e-3)
        self.assertAlmostEqual(result[0][1], 365.311, delta=1e-3)
        self.assertAlmostEqual(result[1][0], 395.934, delta=1e-3)
        self.assertAlmostEqual(result[1][1], 284.689, delta=1e-3)

    def test_angle_two_points(self):
        self.assertAlmostEqual(loc_utils.get_angle_two_points(1.0, 1.0, 2.0, 2.0), np.pi/4, delta=1e-6)
        self.assertAlmostEqual(loc_utils.get_angle_two_points(1.0, 1.0, 2.0, 0.0), (-np.pi/4)%(2*np.pi), delta=1e-6)
        self.assertAlmostEqual(loc_utils.get_angle_two_points(1.0, 1.0, 0.0, 2.0), 3*np.pi/4, delta=1e-6)
        self.assertAlmostEqual(loc_utils.get_angle_two_points(1.0, 1.0, 0.0, 0.0), (-3*np.pi/4)%(2*np.pi), delta=1e-6)

    def test_angle_three_points(self):
        self.assertAlmostEqual(loc_utils.get_angle_three_points(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.0, delta=1e-6)
        self.assertAlmostEqual(loc_utils.get_angle_three_points(1.0, 1.0, 0.0, 0.0, 1.0, 1.0), 0.0, delta=1e-6)
        self.assertAlmostEqual(loc_utils.get_angle_three_points(10.0, -1.0, 0.0, 0.0, 10.0, -1.0), 0.0, delta=1e-6)

        self.assertAlmostEqual(loc_utils.get_angle_three_points(1.0, 0.0, 0.0, 0.0, -1.0, 0.0), np.pi, delta=1e-6)

        self.assertAlmostEqual(loc_utils.get_angle_three_points(4.0, 6.0, 3.0, 4.0, 5.0, 5.0), 0.64350110879, delta=1e-6)
        self.assertAlmostEqual(loc_utils.get_angle_three_points(5.0, 5.0, 3.0, 4.0, 4.0, 6.0), (-0.64350110879)%(2*np.pi), delta=1e-6)
        
