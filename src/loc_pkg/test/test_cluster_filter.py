import sys
sys.path.append("../")
from loc_pkg.localization.Point import Point
from loc_pkg.localization.ClusterFilter import ClusterFilter, Cluster
import numpy as np

import unittest

class TestClusterFilter(unittest.TestCase):
    def test_input_data(self):
        f = ClusterFilter()
        self.assertRaises(AssertionError, lambda: f.input_data(None))
        self.assertRaises(AssertionError, lambda: f.input_data([]))
        self.assertRaises(AssertionError, lambda: f.input_data([12.0]))
        d = [Cluster(), Cluster()]
        d[0].label = "test1"
        d[1].label = "test2"
        f.input_data(d)
        self.assertEqual(f.clusters[0].label, "test1")
        self.assertEqual(f.clusters[1].label, "test2")
    
    def test_remove_by_count(self):
        f = ClusterFilter()
        f.set_config(4, 5.0)
        d = [Cluster(), Cluster()]
        d[0].points = [Point(), Point()]
        d[1].points = [Point(), Point(), Point(), Point(), Point(), Point()]
        f.input_data(d)
        f.filter_by_point_count()
        self.assertEqual(f.clusters[0], d[0])

    def test_remove_by_count(self):
        f = ClusterFilter()
        f.set_config(4, 5.0)
        d = [Cluster(), Cluster()]
        d[0].points = [Point(), Point()]
        d[0].points[0].pos_x = 0.0
        d[0].points[0].pos_y = 0.0
        d[0].points[1].pos_x = 6.0
        d[0].points[1].pos_y = 6.0
        d[1].points = [Point(), Point()]
        d[1].points[0].pos_x = 0.0
        d[1].points[0].pos_y = 0.0
        d[1].points[1].pos_x = 3.0
        d[1].points[1].pos_y = 1.0
        f.input_data(d)
        f.filter_by_size()
        self.assertEqual(len(f.clusters), 1)
        self.assertEqual(f.clusters[0].points[0], d[1].points[0])
        self.assertEqual(f.clusters[0].points[1], d[1].points[1])



if __name__ == "__main__":
    unittest.main()