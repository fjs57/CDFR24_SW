import sys
sys.path.append("../")
from loc_pkg.localization.Point import Point
from loc_pkg.localization.Cluster import Cluster

import numpy as np

import unittest

class TestCluster(unittest.TestCase):

    def point_is_in_cluster(cluster:Cluster, point:Point):
        for p in cluster.points:
            if p == point:
                return True
        return False

    def test_insert_point(self):
        c = Cluster()
        p1 = Point()
        p1.pos_x = 1
        p1.pos_y = 1
        p1.compute_polar()
        c.insert_point(p1)
        self.assertTrue(TestCluster.point_is_in_cluster(c, p1)) # checks that the point is well inserted

        p2 = Point()
        p2.pos_x = 5
        p2.pos_y = 5
        p2.compute_polar()
        p1 = p2
        self.assertFalse(TestCluster.point_is_in_cluster(c, p2)) # checks that the point inserted is not linled with its first definition

        self.assertRaises(AssertionError, lambda : c.insert_point(None))
        self.assertRaises(AssertionError, lambda : c.insert_point(12))

    def test_compute_cluster_position(self):
        c = Cluster()

        self.assertRaises(AssertionError, lambda : c.compute_cluster_position())

        p1 = Point()
        p1.pos_r = 1.0
        p1.pos_th = 0.1
        p1.compute_cartesian()

        c.insert_point(p1)

        c.compute_cluster_position()
        c.center.compute_cartesian()

        self.assertAlmostEqual(c.center.pos_x,  p1.pos_x, delta=1e-3)
        self.assertAlmostEqual(c.center.pos_y,  p1.pos_y, delta=1e-3)

        p1.pos_th = -0.1
        p1.compute_cartesian()

        c.insert_point(p1)
        c.compute_cluster_position()

        self.assertAlmostEqual(c.center.pos_x, 1.0, delta=1e-2)
        self.assertAlmostEqual(c.center.pos_y, 0.0, delta=1e-3)

    def test_get_size(self):
        c = Cluster()
        c.points = [Point(), Point(), Point()]
        c.points[0].pos_x = 0.0
        c.points[0].pos_y = 0.0
        c.points[1].pos_x = 4.0
        c.points[1].pos_y = 0.0
        c.points[2].pos_x = 0.0
        c.points[2].pos_y = 3.0
        self.assertAlmostEqual(c.get_size(), 5.0, delta=1e-6)

        


if __name__ == "__main__":
    unittest.main()