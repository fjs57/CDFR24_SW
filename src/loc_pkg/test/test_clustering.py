import random
import sys
sys.path.append("../")
from loc_pkg.localization.Point import Point
from loc_pkg.localization.Cluster import Cluster
from loc_pkg.localization.Clustering import Clustering

import numpy as np

import unittest

class TestClustering(unittest.TestCase):

    def construct_dataset()->list[Point]:
        points = [Point(), Point(), Point(), Point(), Point(), Point(), Point()]
        offsets = [[0.0,0.0], [1.0,1.0], [-1.0, -1.0]]
        cluster_pos = [[10.0, 0.0], [0.0, 10.0]]
        for i in range(2):
            for j in range(3):
                offset = offsets[j]
                pos = cluster_pos[i]
                index = i*3+j
                points[index].pos_x = pos[0] + offset[0]
                points[index].pos_y = pos[1] + offset[1]
                points[index].compute_polar()
        points[6].pos_x = 0.0
        points[6].pos_y = 0.0
        points[6].compute_polar()
        return points

    def test_set_input(self):
        dataset = TestClustering.construct_dataset()
        c = Clustering(0.3, 3)
        c.set_input(dataset)

        self.assertEqual(len(dataset), len(c.points))

        for i in range(len(dataset)):
            self.assertEqual(dataset[i], c.points[i])

        self.assertRaises(AssertionError, lambda: c.set_input(None))

    def test_execute_clustering(self):
        dataset = TestClustering.construct_dataset()
        c = Clustering(2, 3)
        c.set_input(dataset)
        c.execute_clustering()
        self.assertEqual(c.points[0].cluster_id, 0)
        self.assertEqual(c.points[1].cluster_id, 0)
        self.assertEqual(c.points[2].cluster_id, 0)
        self.assertEqual(c.points[3].cluster_id, 1)
        self.assertEqual(c.points[4].cluster_id, 1)
        self.assertEqual(c.points[5].cluster_id, 1)
        self.assertEqual(c.points[6].cluster_id, None)

    # def test_create_clusters(self):
    #     dataset = TestClustering.construct_dataset()
    #     random.shuffle(dataset)
    #     centers = [Point(), Point()]
    #     cluster_pos = [[10.0, 0.0], [0.0, 10.0]]
    #     for i in range(2):
    #         pos = cluster_pos[i]
    #         center = centers[i]
    #         center.pos_x = pos[0]
    #         center.pos_y = pos[1]
    #         center.compute_polar()
    #     c = Clustering(2, 3)
    #     c.set_input(dataset)
    #     c.execute_clustering()
    #     c.create_clusters()
    #     self.assertEqual(len(c.clusters), 2)
    #     for i in range(2):
    #         self.assertEqual(c.clusters[i].center, centers[i])
    #     q = c.get_clusters()
    #     self.assertEqual(len(q), 2)
    #     for i in range(2):
    #         self.assertEqual(q[i].center, centers[i])

        

        


if __name__ == "__main__":
    unittest.main()