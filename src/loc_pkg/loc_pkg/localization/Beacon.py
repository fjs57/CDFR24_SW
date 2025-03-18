import copy

import numpy as np
from .Point import Point
from .Position import Position
from .PositionPredictor import PositionPredictor
from .Cluster import Cluster
from .loc_utils import cicle_intersections

class Beacon:
    position : Point
    predicted_position : Point
    id : int
    nearest_cluster : Cluster

    def __init__(self, id:int, pos:Point):
        self.id = id
        self.position = pos

    def compute_predicted_position(self, pred:PositionPredictor):
        self.predicted_position = pred.transform_from_odom_to_robot_ref(copy.deepcopy(self.position))

    def get_distance_to_point(self, point:Point):
        dist, _ = self.predicted_position.get_distance_and_angle_to_point(point)
        return dist
    
    def get_nearest_cluster(self, clusters:list[Cluster], pred:PositionPredictor):
        err_d, err_a = pred.get_prediction_error()
        max_distance = err_d + err_a * np.sqrt(self.position.pos_x**2+self.position.pos_y**2) + 0.1

        best_cluster = None
        best_distance = 999.0
        for cluster in clusters:
            dist, _ = self.position.get_distance_and_angle_to_point(cluster.center)
            if dist > max_distance:
                continue
            if dist < best_distance:
                best_cluster = cluster
                best_distance = dist
        self.nearest_cluster = best_cluster

    def compute_position_candidates(self, other:"Beacon"):

        x1, y1 = self.position.pos_x, self.position.pos_y
        x2, y2 = other.position.pos_x, other.position.pos_y

        r1 = np.sqrt(self.nearest_cluster.center.pos_x**2 + self.nearest_cluster.center.pos_y**2) + 0.05*2/3
        r2 = np.sqrt(other.nearest_cluster.center.pos_x**2 + other.nearest_cluster.center.pos_y**2) + 0.05*2/3

        return cicle_intersections(x1, y1, r1, x2, y2, r2)

