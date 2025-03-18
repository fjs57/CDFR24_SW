import rclpy
from rclpy.node import Node

import numpy as np
from sklearn.cluster import DBSCAN

from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32

def cicle_intersections(x1, y1, r1, x2, y2, r2):
    cx = x2 - x1
    cy = y2 - y1
    R = np.sqrt(cx**2+cy**2)
    
    if (abs(r1-r2) > R) and (R <= (r1 + r2)):
        return [] # no solutions
        
    a = r1**2-r2**2
    b = a/(2*R**2)
    c = np.sqrt(2*(r1**2 + r2**2)/R**2 - a**2/R**4 - 1)

    fx = (x1+x2)/2 + b * cx
    gx = c * cy / 2
    ix1 = fx + gx
    ix2 = fx - gx

    fy = (y1+y2)/2 + b * cy
    gy = c * cx / 2
    iy1 = fy + gy
    iy2 = fy - gy

    if gx == 0 and gy == 0: # tangeant circles
        return [[ix1, iy1]]
    
    return [[ix1, iy1], [ix2, iy2]] 

def distance_points(x1, y1, x2, y2):
    return np.sqrt((x1-x2)**2+(y1-y2)**2)



class PositionresolverNode(Node):

    landmarks : PointCloud
    landmark_distances = list[float]
    landmarks_positions = list[list[float]]
    indexes_sequence = list[list[int]]
    possible_positions = list[list[float]]

    param_landmark_1_pos_x : float = 0.0
    param_landmark_1_pos_y : float = -1.5
    param_landmark_2_pos_x : float = 1.0
    param_landmark_2_pos_y : float = 1.5
    param_landmark_3_pos_x : float = -1.0
    param_landmark_3_pos_y : float = 1.5
    param_landmark_distance_error_tolerance : float = 0.1
    param_terrain_max_x : float = 1.5
    param_terrain_min_x : float = -1.5
    param_terrain_max_y : float = 1.0
    param_terrain_min_y : float = -1.0

    def __init__(self):
        super().__init__('absodom_clustering_node')

        self.lidar_subscription = self.create_subscription(
            PointCloud,
            'landmark_positions',
            self.receive_landmarks,
            10
        )

        self.get_logger().info("start node")
        # self.cluster_publisher = self.create_publisher(
        #     PointCloud,
        #     'cartesian_scan',
        #     10
        # )

        self.landmark_distances = []

        self.landmarks_positions = [
            [self.param_landmark_1_pos_x, self.param_landmark_1_pos_y],
            [self.param_landmark_2_pos_x, self.param_landmark_2_pos_y],
            [self.param_landmark_3_pos_x, self.param_landmark_3_pos_y],
        ]

        self.indexes_sequence = [
            [0,1], [1,2], [2,0]
        ]

        for indexes in self.indexes_sequence:
            A = self.landmarks_positions[indexes[0]]
            B = self.landmarks_positions[indexes[1]]

            self.landmark_distances.append(np.sqrt( (A[0]-B[0])**2 + (A[1]-B[1])**2 ))

    def receive_landmarks(self, landmarks:PointCloud):
        # if not self.accept_new_scan:
        #     return
        self.landmarks = []
        for point in landmarks.points:
            self.landmarks.append([point.x, point.y])
        # self.accept_new_scan = False
        # self.process()
        self.new_process()

    def new_process(self):
        number_of_landmarks_found = len(self.landmarks)
        self.get_logger().info("number of landmarks before filtering : {}".format(number_of_landmarks_found) )

        if len(self.landmarks) < 2:
            return
        
        if len(self.landmarks) == 2:
            pass

    def get_distance_landmarks(self, idA, idB):
        pass

    def find_landmarks(self):
        pass

        


    def process(self):
        
        to_be_kept = []
        new_landmarks = []
        self.possible_positions = []

        already_done = dict()

        for iA, pA in enumerate(self.landmarks):

            already_done

            for iB, pB in enumerate(self.landmarks):
                if iA == iB:
                    continue

                dist = np.sqrt( (pA[0]-pB[0])**2 + (pA[1]-pB[1])**2 )

                for arrangement, element in enumerate(self.landmark_distances):
                    if abs(dist - element) < self.param_landmark_distance_error_tolerance:
                        
                        if iA not in to_be_kept:
                            to_be_kept.append(iA)
                            new_landmarks.append(pA)

                        if iB not in to_be_kept:
                            to_be_kept.append(iB)
                            new_landmarks.append(pB)

                        idthA, idthB = self.indexes_sequence[arrangement]                  

                        self.compute_possible_positions(iA, iB, idthA, idthB)
                        self.compute_possible_positions(iB, iA, idthA, idthB)

                        
        self.landmarks = new_landmarks
        # self.get_logger().info("-----------------------")
        # for id, landmark in enumerate(self.landmarks):
        #     self.get_logger().info(str(id)+ " : " +str(landmark))

        self.filter_possible_positions()
        self.get_position_estimate()

    def compute_possible_positions(self, iA, iB, idthA, idthB):

        realA = np.array(self.landmarks_positions[idthA])
        realB = np.array(self.landmarks_positions[idthB])
        
        pA = self.landmarks[iA]
        rA = np.sqrt(pA[0]**2+pA[1]**2)

        pB = self.landmarks[iB]
        rB = np.sqrt(pB[0]**2+pB[1]**2)

        self.possible_positions += cicle_intersections(realA[0], realA[1], rA, realB[0], realB[1], rB)
    
    def filter_possible_positions(self):

        to_be_kept = []

        for pos in self.possible_positions:
            if self.param_terrain_min_x <= pos[0] <= self.param_terrain_max_x :
                if self.param_terrain_min_y <= pos[1] <= self.param_terrain_max_y :
                    to_be_kept.append(pos)

        self.possible_positions = to_be_kept


    def get_position_estimate(self):

        current_best = 999.9
        best_positions = None

        for A in self.possible_positions:
            for B in self.possible_positions:
                if A == B :
                    continue
                for C in self.possible_positions :
                    if C in [A, B]:
                        continue
                    metric = max(
                        abs(A[0] - B[0]) + abs(A[1] - B[1]),
                        abs(A[0] - C[0]) + abs(A[1] - C[1]),
                        abs(C[0] - B[0]) + abs(C[1] - B[1]),
                    )

                    if metric < current_best:
                        current_best = metric
                        best_positions = [A, B, C]

        self.get_logger().info(str(self.possible_positions))





        


def main(args=None):
    rclpy.init(args=args)
    resolver_node = PositionresolverNode()
    rclpy.spin(resolver_node)
    resolver_node.destroy_node()
    rclpy.shutdown()
    # print(cicle_intersections(-1.5,0, 2, 1.5, 1, 1.6))

if __name__ == '__main__':
    main()
