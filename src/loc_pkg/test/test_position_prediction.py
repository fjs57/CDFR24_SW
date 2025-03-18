import sys
sys.path.append("../")
from loc_pkg.localization.PositionPredictor import Position, PositionPredictor
from loc_pkg.localization.Point import Point
import numpy as np

import unittest

class TestPositionPrediction(unittest.TestCase):
    def test_prediction(self):
        c = Position(3.0, 3.0, 0.0)
        i = Position(2.0, 4.0, 0.0)
        r = Position(1.0, 2.0, 0.0)

        p = PositionPredictor()
        p.set_odom_position(i)
        p.set_lidar_position(r)
        p.set_odom_position(c)

        n = p.get_predicted_position()

        self.assertAlmostEqual(n.x, 2.0, delta=1e-6)
        self.assertAlmostEqual(n.y, 1.0, delta=1e-6)
        self.assertAlmostEqual(n.theta, 0.0, delta=1e-6)

    def test_prediction_with_rotation_initial(self):
        c = Position(3.0, 3.0, np.pi/2)
        i = Position(2.0, 4.0, np.pi/2)
        r = Position(1.0, 2.0, 0.0)

        p = PositionPredictor()
        p.set_odom_position(i)
        p.set_lidar_position(r)
        p.set_odom_position(c)

        n = p.get_predicted_position()

        self.assertAlmostEqual(n.x, 2.0, delta=1e-6)
        self.assertAlmostEqual(n.y, 3.0, delta=1e-6)
        self.assertAlmostEqual(n.theta, 0.0, delta=1e-6)

    def test_prediction_with_rotation_current(self):
        c = Position(3.0, 3.0, np.pi/2)
        i = Position(2.0, 4.0, 0)
        r = Position(1.0, 2.0, 0.0)

        p = PositionPredictor()
        p.set_odom_position(i)
        p.set_lidar_position(r)
        p.set_odom_position(c)

        n = p.get_predicted_position()

        self.assertAlmostEqual(n.x, 2.0, delta=1e-6)
        self.assertAlmostEqual(n.y, 1.0, delta=1e-6)
        self.assertAlmostEqual(n.theta, np.pi/2, delta=1e-6)

    def test_transform_0(self):
        c = Position(3.0, 3.0, 0.0)
        i = Position(2.0, 4.0, 0.0)
        r = Position(1.0, 2.0, 0.0)

        p = PositionPredictor()
        p.set_odom_position(i)
        p.set_lidar_position(r)
        p.set_odom_position(c)

        # position at this point will be [2.0, 1.0], th=0.0

        ptt = Point()
        ptt.pos_x = -2.0
        ptt.pos_y = 3.0
        pat = p.transform_from_odom_to_robot_ref(ptt)

        self.assertAlmostEqual(pat.pos_x, -4.0, delta=1e-6)
        self.assertAlmostEqual(pat.pos_y, 2.0, delta=1e-6)

    def test_transform_1(self):
        c = Position(3.0, 3.0, np.pi/2)
        i = Position(2.0, 4.0, 0.0)
        r = Position(1.0, 2.0, 0.0)

        p = PositionPredictor()
        p.set_odom_position(i)
        p.set_lidar_position(r)
        p.set_odom_position(c)

        # position at this point will be [2.0, 1.0], th=0.0

        ptt = Point()
        ptt.pos_x = -2.0
        ptt.pos_y = 3.0
        pat = p.transform_from_odom_to_robot_ref(ptt)

        self.assertAlmostEqual(pat.pos_x, 2.0, delta=1e-6)
        self.assertAlmostEqual(pat.pos_y, 4.0, delta=1e-6)



if __name__ == "__main__":
    unittest.main()