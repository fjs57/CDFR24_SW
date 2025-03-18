import sys
sys.path.append("../")
from loc_pkg.localization.PositionPredictor import Position
import numpy as np

import unittest

class TestPosition(unittest.TestCase):
    def test_sub(self):
        a = Position(1.0, 0.0, 0.0)
        b = Position(2.0, 3.0, np.pi/2)
        c = a - b
        self.assertAlmostEqual(c.x, a.x-b.x, delta=1e-6)
        self.assertAlmostEqual(c.y, a.y-b.y, delta=1e-6)
        self.assertAlmostEqual(c.theta, a.theta-b.theta, delta=1e-6)

    def test_add(self):
        a = Position(1.0, 0.0, 0.0)
        b = Position(2.0, 3.0, np.pi/2)
        c = a + b
        self.assertAlmostEqual(c.x, a.x+b.x, delta=1e-6)
        self.assertAlmostEqual(c.y, a.y+b.y, delta=1e-6)
        self.assertAlmostEqual(c.theta, a.theta+b.theta, delta=1e-6)

    def test_rot_around_point(self):
        a = Position(2.0, 3.0, 0.0)
        c = Position(0.0, 1.0, 0.0)
        a.rotate_around_point(c, -np.pi/2)
        self.assertAlmostEqual(a.x, 2.0, delta=1e-6)
        self.assertAlmostEqual(a.y, -1.0, delta=1e-6)
        self.assertAlmostEqual(a.theta, -np.pi/2, delta=1e-6)

if __name__ == "__main__":
    unittest.main()