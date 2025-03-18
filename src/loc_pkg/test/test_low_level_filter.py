import sys
sys.path.append("../")
from loc_pkg.localization.LowLevelFilter import LowLevelFilter, LaserScan
import numpy as np

import unittest

class TestLLFilter(unittest.TestCase):
    def create_dataset():
        scan = LaserScan()
        scan.angle_min = 0.0
        scan.angle_increment = np.pi/2
        scan.ranges = [0.1, 1.0, 10.0, 2.0]
        return scan
    
    def test_filter(self):
        l = LowLevelFilter()
        s = TestLLFilter.create_dataset()

        self.assertRaises(AssertionError, lambda: l.process_data(s))

        l.set_config(0.2, 5)
        p = l.process_data(s)

        self.assertEqual(len(p), 2)
        self.assertAlmostEqual(p[0].pos_r, 1.0, delta=1e-6)
        self.assertAlmostEqual(p[1].pos_r, 2.0, delta=1e-6)

if __name__ == "__main__":
    unittest.main()