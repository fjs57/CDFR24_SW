import rclpy
from rclpy.node import Node

import numpy as np
from sklearn.cluster import DBSCAN

from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from tf2_ros.transform_broadcaster import TransformBroadcaster

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class Cluster:
    points : list[list[float]] = []
    angles : list[float] = []
    distances : list[float] = []
    center : list[float] = []
    distance_robot_center : float = None
    angle_from_robot : float = None

    def __init__(self):
        self.points = []
        self.angles = []
        self.distances = []
        self.center = None

    def insert_point(self, point:list[float]):
        self.points.append(point)

    def insert_polar(self, polar:list[float]):
        self.angles.append(polar[0])
        self.distances.append(polar[1])

    def get_size(self):
        points = np.array(self.points)
        stddev = np.std(points, axis=0)
        self.size = np.sqrt(stddev[0]**2+stddev[1]**2) * 3 # 3 std dev = 95% of samples
        return self.size
    
    def get_agregated_angle(self):
        self.angle_from_robot = np.mean(self.angles)
        return self.angle_from_robot
    
    def get_offset(self, offset_distance:float):
        self.get_agregated_angle()
        return offset_distance * np.array([np.cos(self.angle_from_robot), np.sin(self.angle_from_robot)])
    
    def get_center(self, offset_distance:float):
        mean_pose = np.mean(self.points, axis=0)
        self.center = mean_pose + self.get_offset(offset_distance)
        return self.center
    
    def get_distance(self, offset_distance):
        self.get_center(offset_distance)
        self.distance_robot_center = np.sqrt(self.center[0]**2+self.center[1]**2)
        return self.distance_robot_center

class Landmark():
    position : list[float] = []

    def __init__(self):
        super().__init__()

    def get_distance_to_point(self, other:list[float]):
        return np.sqrt( (self.position[0]-other[0])**2 + (self.position[1]-other[1])**2)
    
    def get_distance_to_landmark(self, other):
        return self.get_distance_to_point(other.position)
    
    def get_angle_between_landmarks(self, A, B):
        ang1 = np.arctan2(B.position[1] - self.position[1], B.position[0] - self.position[0])
        ang2 = np.arctan2(A.position[1] - self.position[1], A.position[0] - self.position[0])
        ang = ang1 - ang2
        return ang
    
    def get_heading(self, pos:list[float]):
        return np.arctan2(self.position[1]-pos[1], self.position[0]-pos[0])

class PossibleLandmark(Landmark):

    cluster : Cluster

    def __init__(self, cluster:Cluster, center_offset:float):
        super().__init__()
        self.cluster = cluster
        self.position = cluster.get_center(center_offset)
        self.center_offset = center_offset

    def get_distance(self):
        return self.cluster.get_distance(self.center_offset)

    @staticmethod
    def compute_circle_intersections(pla, plb, rla:Landmark, rlb:Landmark):
        r1 = pla.get_distance()
        r2 = plb.get_distance()
        x1, y1 = rla.position
        x2, y2 = rlb.position
        
        cx = x2 - x1
        cy = y1 - y1
        R = np.sqrt(cx**2+cy**2)
        
        if (abs(r1-r2) > R) and (R <= (r1 + r2)):
            print("no solutions")
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

        return [[ix1, iy1], [ix2, iy2]] 
    
    def compute_robot_heading(self, real:Landmark, position:list[float]):
        direction = real.get_heading(position)
        vision = self.cluster.angle_from_robot
        return (direction - vision + np.pi) % (2*np.pi) - np.pi


    
    



class ClusteringNode(Node):

    accept_new_scan : bool = None
    current_scan_polar : LaserScan = []
    current_scan_cartesian : list[list[float]] = []
    current_scan_cartesian_angles : list[float] = []
    current_clusters : dict[Cluster]
    real_landmarks : list[Landmark] = []
    real_landmarks_distances_sequence : dict = {0:1,1:2,2:0}
    real_landmarks_distances : list[float] = []
    real_landmark_angle : float
    possible_landmarks : list[PossibleLandmark] = []
    identified_landmarks : list[PossibleLandmark] = []
    position : list[float] = []
    heading : float = None
    tf_broadcaster : TransformBroadcaster

    param_clustering_epsilon : float = 0.15
    param_clustering_min_samples : int = 3
    param_filter_min_distance_to_bot : float = 0.2
    param_filter_max_distance_to_bot : float = 5.0
    param_max_cluster_expansion : float = 0.15
    param_landmark_radius : float = 0.05
    param_landmark_offset_scale : float = 0.70

    param_landmark_0_pos_x : float = 0
    param_landmark_0_pos_y : float = -1.5

    param_landmark_1_pos_x : float = 1.0
    param_landmark_1_pos_y : float = 1.5

    param_landmark_2_pos_x : float = -1.0
    param_landmark_2_pos_y : float = 1.5

    param_landmark_distance_error_tolerance : float = 0.5
    param_landmark_angle_error_tolerance : float = 0.1
    param_terrain_max_x : float = 2
    param_terrain_min_x : float = -2
    param_terrain_max_y : float = 2
    param_terrain_min_y : float = -2
    param_initialization_position_x : float = 0.0
    param_initialization_position_y : float = 0.0
    param_initialization_position_r : float = 0.0

    def __init__(self):
        super().__init__('absodom_clustering_node')

        self.real_landmarks = [Landmark() for _ in range(3)]
        self.real_landmarks[0].position = [self.param_landmark_0_pos_x, self.param_landmark_0_pos_y]
        self.real_landmarks[1].position = [self.param_landmark_1_pos_x, self.param_landmark_1_pos_y]
        self.real_landmarks[2].position = [self.param_landmark_2_pos_x, self.param_landmark_2_pos_y]
        self.real_landmarks_distances = [ self.real_landmarks[A].get_distance_to_landmark(self.real_landmarks[B]) for A, B in self.real_landmarks_distances_sequence.items() ]
        self.real_landmark_angle = self.real_landmarks[1].get_angle_between_landmarks(self.real_landmarks[0],self.real_landmarks[2])

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.receive_scan_callback,
            10
        )
        self.cluster_publisher = self.create_publisher(
            PointCloud,
            'landmark_positions',
            10
        )
        # self.odometry_publisher = self.create_publisher(
        #     Odometry,
        #     'lidar_odom',
        #     10
        # )

        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            'lidar_pose',
            10
        )

        self.accept_new_scan = True

        self.tf_broadcaster = TransformBroadcaster(self)

    def output_pose(self):
        quat = quaternion_from_euler(0.0, 0.0, self.heading)
        m = PoseWithCovarianceStamped()

        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"

        m.pose.pose.position.x = self.position[0]
        m.pose.pose.position.y = self.position[1]
        m.pose.pose.position.z = 0.0
        m.pose.pose.orientation.x = 0.0
        m.pose.pose.orientation.y = 0.0
        m.pose.pose.orientation.z = quat[2]
        m.pose.pose.orientation.w = quat[3]

        m.pose.covariance = [
            0.007, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.007, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.008726 
        ]

        self.pose_publisher.publish(m)

    def output_odom(self):
        quat = quaternion_from_euler(0.0, 0.0, self.heading)
        m = Odometry()

        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "world"
        m.child_frame_id = "lidar_odom"
        m.pose.pose.position.x = self.position[0]
        m.pose.pose.position.y = self.position[1]
        m.pose.pose.position.z = 0.0
        m.pose.pose.orientation.x = 0.0
        m.pose.pose.orientation.y = 0.0
        m.pose.pose.orientation.z = quat[2]
        m.pose.pose.orientation.w = quat[3]

        self.odometry_publisher.publish(m)

    def output_tf(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'lidar_odom'

        t.transform.translation.x = float(self.position[0])
        t.transform.translation.y = float(self.position[1])
        t.transform.translation.z = float(0)
        quat = quaternion_from_euler(
            float(0), float(0), float(self.heading))
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

    def receive_scan_callback(self, scan:LaserScan):
        # if not self.accept_new_scan:
        #     return
        self.current_scan_polar = scan
        # self.accept_new_scan = False
        self.process()

    def process(self):
        self.polar_to_cartesian()
        self.clusterization_process()
        # self.output_clusters()
        self.filter_clusters()
        if len(self.current_clusters) < 3:
            self.get_logger().info("dismissed not enough clusters")
            return
        self.identify_landmarks()
        if self.identified_landmarks == None:
            self.get_logger().info("dismissed indetification failed")
            return
        self.output_landmarks()
        status = self.compute_possible_positions()
        if not status :
            return
        self.compute_yaw()
        if self.heading == []:
            return
        # self.output_tf()
        # self.output_odom()
        self.output_pose()
        

    def polar_to_cartesian(self):
        start_angle = self.current_scan_polar.angle_min
        angle_increment = self.current_scan_polar.angle_increment
        samples = self.current_scan_polar.ranges

        self.current_scan_cartesian = []
        self.current_scan_cartesian_angles = []

        for index, sample in enumerate(samples):
            if sample < self.param_filter_min_distance_to_bot:
                continue
            if sample > self.param_filter_max_distance_to_bot:
                continue

            angle = start_angle + index * angle_increment

            x = sample * np.cos(angle)
            y = sample * np.sin(angle)

            # self.current_scan_cartesian[0].append(x)
            # self.current_scan_cartesian[1].append(y)
            self.current_scan_cartesian.append([x, y])
            self.current_scan_cartesian_angles.append([angle, sample])

        self.current_scan_cartesian = np.array(self.current_scan_cartesian)
    


    def clusterization_process(self):
        db = DBSCAN(eps=self.param_clustering_epsilon, min_samples=self.param_clustering_min_samples).fit(self.current_scan_cartesian)

        self.current_clusters = dict()
        for index, cluster_id in enumerate(db.labels_):
            if cluster_id not in self.current_clusters.keys():
                self.current_clusters[cluster_id] = Cluster()
            point = self.current_scan_cartesian[index]
            polar = self.current_scan_cartesian_angles[index]
            self.current_clusters[cluster_id].insert_point(point)
            self.current_clusters[cluster_id].insert_polar(polar)

    def filter_clusters(self):
        to_be_deleted = []
        for key, cluster in self.current_clusters.items():
            size = cluster.get_size()
            if size > self.param_max_cluster_expansion:
                to_be_deleted.append(key)

        for key in to_be_deleted:
            self.current_clusters.pop(key)
        
    def output_clusters(self):
        self.get_logger().info("Output clusters")

        cloud = PointCloud()
        cloud.header.frame_id = "laser_frame"
        cloud.channels = [ChannelFloat32()]
        cloud.channels[0].name = "Cluster ID"
        cloud.points = []

        for id, cluster in self.current_clusters.items():
            cloud.channels[0].values.append(id)
            point = Point32()
            center = cluster.get_center(self.param_landmark_radius)
            point.x = center[0]
            point.y = center[1]
            cloud.points.append(point)

        self.cluster_publisher.publish(cloud)

    def output_landmarks(self):
        self.get_logger().info("Output landmarks")

        cloud = PointCloud()
        cloud.header.frame_id = "laser_frame"
        cloud.channels = [ChannelFloat32()]
        cloud.channels[0].name = "Landmark ID"
        cloud.points = []

        for id, landmark in enumerate(self.identified_landmarks):
            cloud.channels[0].values.append(id)
            point = Point32()
            point.x = landmark.position[0]
            point.y = landmark.position[1]
            cloud.points.append(point)

        self.cluster_publisher.publish(cloud)

    def compute_error_on_distance(self, landmarks:list[PossibleLandmark]):
        seq = self.real_landmarks_distances_sequence
        max_error = 0
        for i in range(len(seq)):
            iA = list(seq.keys())[i]
            iB = seq[iA]
            expected_distance = self.real_landmarks_distances[i]
            A = landmarks[iA]
            B = landmarks[iB]
            dist = A.get_distance_to_landmark(B)
            error = abs(expected_distance - dist)
            max_error = max(max_error, error)
        return max_error
    
    def compute_error_on_angles(self, landmarks:list[PossibleLandmark]):
        A = landmarks[0]
        B = landmarks[1]
        C = landmarks[2]
        angle = B.get_angle_between_landmarks(A, C) % (2*np.pi)
        error = abs(angle-self.real_landmark_angle)
        return error

    def identify_landmarks(self):
        # search the best triplet

        # self.get_logger().info("idntification"+str(self.current_clusters))

        best_landmarks = []
        best_distance_error = 999.9

        for iA, cA in self.current_clusters.items():
            for iB, cB in self.current_clusters.items():
                if iB == iA:
                    continue
                for iC, cC in self.current_clusters.items():
                    if iC in [iA, iB]:
                        continue
                    A = PossibleLandmark(cA, self.param_landmark_radius*self.param_landmark_offset_scale)
                    B = PossibleLandmark(cB, self.param_landmark_radius*self.param_landmark_offset_scale)
                    C = PossibleLandmark(cC, self.param_landmark_radius*self.param_landmark_offset_scale)

                    landmarks = [A, B, C]
                    error_distance = self.compute_error_on_distance(landmarks)
                    error_angle = self.compute_error_on_angles(landmarks)

                    if error_angle >= self.param_landmark_angle_error_tolerance:
                        # self.get_logger().info("\t  NOK ANGLE : {}".format(error_angle*180*3.1415))
                        continue

                    if error_distance < best_distance_error:
                        best_distance_error = error_distance
                        best_landmarks = [A, B, C]

        if best_distance_error <= self.param_landmark_distance_error_tolerance:
            self.identified_landmarks = best_landmarks
            # self.get_logger().info("\t> OK")
        else :
            # self.get_logger().info("\t  NOK DISTANCE : {}".format(best_distance_error))
            self.identified_landmarks = None

    def test_possible_position(self, pos):
        for i in range(3):
            det = self.identified_landmarks[i]
            real = self.real_landmarks[i]
            error = abs(real.get_distance_to_point(pos) - det.get_distance())
            if error > self.param_landmark_distance_error_tolerance:
                return False
        return True
    
    def compute_possible_positions(self):
        possible_positions = []
        for start, end in self.real_landmarks_distances_sequence.items():
            tmp_possible_positions = PossibleLandmark.compute_circle_intersections(
                self.identified_landmarks[start],
                self.identified_landmarks[end],
                self.real_landmarks[start],
                self.real_landmarks[end]
            )
            for pos in tmp_possible_positions:
                if self.test_possible_position(pos):
                    possible_positions.append(pos)

        if possible_positions == []:
            return False

        self.position = np.mean(possible_positions, axis=0)
        self.get_logger().info("position :\n\tx:{:.3f}\n\ty:{:.3f}\n\n---------\n".format(self.position[0], self.position[1])) 

        return True

    def compute_yaw(self):
        headings = []
        for i in range(3):
            det = self.identified_landmarks[i]
            real = self.real_landmarks[i]
            headings.append(det.compute_robot_heading(real, self.position))
        self.heading = np.mean(headings)
        self.get_logger().info("heading :\n\tr:{:.3f}".format(self.heading))
        


def main(args=None):
    rclpy.init(args=args)
    clustering_node = ClusteringNode()
    rclpy.spin(clustering_node)
    clustering_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
