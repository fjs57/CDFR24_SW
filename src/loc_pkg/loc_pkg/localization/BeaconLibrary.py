import numpy as np
from .Beacon import Beacon
from .Point import Point
from .PositionPredictor import PositionPredictor, Position
from .Cluster import Cluster

from .loc_utils import get_angle_three_points

class BeaconLibrary:
    count : int
    beacons : list[Beacon]
    position_candidates : list[list[float]]
    diffs : list[list[float]]

    def __init__(self, count:int, positions:list[float]):
        assert(len(positions)==2*count)
        self.count = count
        self.beacons = []
        for i in range(count):
            pos = Point()
            pos.pos_x = positions[2*i]
            pos.pos_y = positions[2*i+1]
            self.beacons.append(Beacon(id=i, pos=pos))

    def transform_to_robot_ref(self, pred:PositionPredictor):
        for beacon in self.beacons:
            beacon.compute_predicted_position(pred)

    def identify_clusters(self, clusters:list[Cluster], pred:PositionPredictor):
        for beacon in self.beacons:
            beacon.get_nearest_cluster(clusters, pred)

    def compute_possible_positions(self) -> list[list[float]]:
        already_computed : dict[int, list[int]] = dict()

        for i in range(self.count):
            already_computed[i] = []
        position_candidates = []
        self.diffs = []

        for id_a, A in enumerate(self.beacons):
            if A.nearest_cluster == None:
                continue
            for id_b, B in enumerate(self.beacons):
                if B.nearest_cluster == None:
                    continue
                if id_a == id_b:
                    continue
                if id_a in already_computed[id_b] or id_b in already_computed[id_a]:
                    continue
                already_computed[id_a].append(id_b)
                already_computed[id_b].append(id_a)
                candidate = A.compute_position_candidates(B)
                if candidate:
                    position_candidates.append(candidate)
                    self.diffs.append([ self.verify_position(x, y, A, B) for x,y in candidate ])

        self.position_candidates = position_candidates
        return position_candidates
    
    def verify_position(self, x:float, y:float, A:Beacon, B:Beacon):
        angle = get_angle_three_points(
            A.position.pos_x, A.position.pos_y,
            x, y,
            B.position.pos_x, B.position.pos_y
        )

        mes = A.nearest_cluster.center.pos_th - B.nearest_cluster.center.pos_th

        diff = (angle-mes+np.pi)%(np.pi*2)-np.pi
        return diff

    def compute_position(self, filter_angle_tolerance:float=0.1, filter_distance_to_mean:float=0.05):
        positions = []
        self.compute_possible_positions()

        for i in range(len(self.position_candidates)):
            for j in range(2):
                if abs(self.diffs[i][j]) > filter_angle_tolerance:
                    continue
                positions.append(self.position_candidates[i][j])
        if len(positions)==0:
            return None
        if len(positions) == 1:
            return positions[0]
        return np.mean(positions, axis=0)
    
    def compute_angle(self, x, y):
        angles = []
        robot = Point()
        robot.pos_x = x
        robot.pos_y = y
        for beacon in self.beacons:
            a = robot.get_distance_and_angle_to_point(beacon.position)
            b = beacon.nearest_cluster.center.pos_th
            angles.append(a-b)
        if angles == []:
            return None
        return np.mean(angles)
    
    def compute_position_and_angle(self):
        position = self.compute_position()
        if not position:
            return None
        angle = self.compute_angle(position[0], position[1])
        if not angle:
            return None
        return Position(position[0], position[1], angle)



    def __str__(self) -> str:
        s = ""
        for b in self.beacons:
            s+="|id={} : {}".format(b.id, b.position.__str__())
        return s
