from .localization.LowLevelFilter import LowLevelFilter
from .localization.Point import Point
from .localization.Clustering import Clustering, Cluster
from .localization.ClusterFilter import ClusterFilter
from .localization.PositionPredictor import PositionPredictor, Position
from .localization.BeaconLibrary import BeaconLibrary

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class LocalizationNode(Node):

    low_level_filter : LowLevelFilter
    position_predictor : PositionPredictor
    clustering : Clustering
    cluster_filter : ClusterFilter
    beacon_library : BeaconLibrary

    param_scan_min_distance : float
    param_scan_max_distance : float
    param_clustering_epsilon : float
    param_clustering_min_point_count : int
    param_cluster_filter_max_point_count : int
    param_cluster_filter_max_size : float
    param_beacon_radius : float
    param_beacon_count : int
    param_beacon_positions : list[float]
    param_beacon_range_correction_factor : float
    param_lidar_range_stddev : float
    param_lidar_angle_stddev : float
    param_odom_distance_error : float
    param_odom_turn_error : float

    def __init__(self):
        super().__init__("localization_node")

        self.init_params()
        self.init_process()
        self.init_subscriptions()
        self.init_publishers()

    def init_params(self): 
        self.parameters = self.declare_parameters(namespace="", parameters=[
            ("scan_min_distance", rclpy.Parameter.Type.DOUBLE),
            ("scan_max_distance", rclpy.Parameter.Type.DOUBLE),
            ("clustering_epsilon", rclpy.Parameter.Type.DOUBLE),
            ("clustering_min_point_count", rclpy.Parameter.Type.INTEGER),
            ("cluster_filter_max_point_count", rclpy.Parameter.Type.INTEGER),
            ("cluster_filter_max_size", rclpy.Parameter.Type.DOUBLE),

            ("lidar_range_stddev", rclpy.Parameter.Type.DOUBLE),
            ("lidar_angle_stddev", rclpy.Parameter.Type.DOUBLE),
            ("odom_distance_error", rclpy.Parameter.Type.DOUBLE),
            ("odom_turn_error", rclpy.Parameter.Type.DOUBLE),

            ("init_x", rclpy.Parameter.Type.DOUBLE),
            ("init_y", rclpy.Parameter.Type.DOUBLE),
            ("init_th", rclpy.Parameter.Type.DOUBLE),

            ("beacon_radius", rclpy.Parameter.Type.DOUBLE),
            ("beacon_count", rclpy.Parameter.Type.INTEGER),
            ("beacon_positions", rclpy.Parameter.Type.DOUBLE_ARRAY),
            ("beacon_range_correction_factor", rclpy.Parameter.Type.DOUBLE)
        ])

        self.param_scan_min_distance = self.get_parameter("scan_min_distance").get_parameter_value().double_value
        self.param_scan_max_distance = self.get_parameter("scan_max_distance").get_parameter_value().double_value
        self.param_clustering_epsilon = self.get_parameter("clustering_epsilon").get_parameter_value().double_value
        self.param_clustering_min_point_count = self.get_parameter("clustering_min_point_count").get_parameter_value().integer_value
        self.param_cluster_filter_max_point_count = self.get_parameter("cluster_filter_max_point_count").get_parameter_value().integer_value
        self.param_cluster_filter_max_size = self.get_parameter("cluster_filter_max_size").get_parameter_value().double_value

        self.param_init_x = self.get_parameter("init_x").get_parameter_value().double_value
        self.param_init_y = self.get_parameter("init_y").get_parameter_value().double_value
        self.param_init_th = self.get_parameter("init_th").get_parameter_value().double_value

        self.param_lidar_range_stddev = self.get_parameter("lidar_range_stddev").get_parameter_value().double_value
        self.param_lidar_angle_stddev = self.get_parameter("lidar_angle_stddev").get_parameter_value().double_value
        self.param_odom_distance_error = self.get_parameter("odom_distance_error").get_parameter_value().double_value
        self.param_odom_turn_error = self.get_parameter("odom_turn_error").get_parameter_value().double_value

        self.param_beacon_radius = self.get_parameter("beacon_radius").get_parameter_value().double_value
        self.param_beacon_count = self.get_parameter("beacon_count").get_parameter_value().integer_value
        self.param_beacon_positions = self.get_parameter("beacon_positions").get_parameter_value().double_array_value
        self.param_beacon_range_correction_factor = self.get_parameter("beacon_range_correction_factor").get_parameter_value().double_value

        self.get_logger().info("param_scan_min_distance : {}".format(self.param_scan_min_distance))
        self.get_logger().info("param_scan_max_distance : {}".format(self.param_scan_max_distance))
        self.get_logger().info("param_clustering_epsilon : {}".format(self.param_clustering_epsilon))
        self.get_logger().info("param_clustering_min_point_count : {}".format(self.param_clustering_min_point_count))
        self.get_logger().info("param_cluster_filter_max_point_count : {}".format(self.param_cluster_filter_max_point_count))
        self.get_logger().info("param_cluster_filter_max_size : {}".format(self.param_cluster_filter_max_size))

        self.get_logger().info("param_init_x : {}".format(self.param_init_x))
        self.get_logger().info("param_init_y : {}".format(self.param_init_y))
        self.get_logger().info("param_init_th : {}".format(self.param_init_th))

        self.get_logger().info("param_lidar_range_stddev : {}".format(self.param_lidar_range_stddev))
        self.get_logger().info("param_lidar_angle_stddev : {}".format(self.param_lidar_angle_stddev))
        self.get_logger().info("param_odom_distance_error : {}".format(self.param_odom_distance_error))
        self.get_logger().info("param_odom_turn_error : {}".format(self.param_odom_turn_error))

        self.get_logger().info("param_beacon_radius : {}".format(self.param_beacon_radius))
        self.get_logger().info("param_beacon_count : {}".format(self.param_beacon_count))
        self.get_logger().info("param_beacon_positions : {}".format(self.param_beacon_positions))
        self.get_logger().info("param_beacon_range_correction_factor : {}".format(self.param_beacon_range_correction_factor))

    def init_subscriptions(self):

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.process_data,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            'diff_cont/odom',
            self.process_odom,
            10
        )

    def init_publishers(self):

        self.filtered_points_publisher = self.create_publisher(
            PointCloud,
            'filtered_points',
            10
        )

        self.clustered_points_publisher = self.create_publisher(
            PointCloud,
            'clustered_points',
            10
        )

        self.clusters_publisher = self.create_publisher(
            PointCloud,
            'clusters',
            10
        )

        self.predicted_beacons_publisher = self.create_publisher(
            PointCloud,
            'predicted_beacons',
            10
        )

    def init_process(self):

        self.position_predictor = PositionPredictor()
        self.position_predictor.set_config(
            self.param_lidar_range_stddev,
            self.param_lidar_angle_stddev,
            self.param_odom_distance_error,
            self.param_odom_turn_error
        )
        self.position_predictor.set_initial_position(
            self.param_init_x,
            self.param_init_y,
            self.param_init_th
        )

        self.low_level_filter = LowLevelFilter()
        self.low_level_filter.set_config(
            self.param_scan_min_distance,
            self.param_scan_max_distance
        )

        self.clustering = Clustering()
        self.clustering.set_config(
            self.param_clustering_epsilon,
            self.param_clustering_min_point_count
        )

        self.cluster_filter = ClusterFilter()
        self.cluster_filter.set_config(
            self.param_cluster_filter_max_point_count,
            self.param_cluster_filter_max_size
        )

        self.beacon_library = BeaconLibrary(
            self.param_beacon_count,
            self.param_beacon_positions
        )
        self.get_logger().info(str(self.beacon_library))
        self.get_logger().info(str(self.beacon_library.beacons[0].position.pos_x))


    def process_data(self, data:LaserScan):

        filtered_points = self.low_level_filter.process_data(data)
        self.publish_filtered_points(filtered_points)

        self.clustering.set_input(filtered_points)
        self.clustering.execute_clustering()
        self.clustering.create_clusters()

        if self.clustering.clusters == None:
            self.get_logger().debug("\tclustering done, cluster count : None")
            return
        else :
            self.get_logger().debug("\tclustering done, cluster count : {}".format(len(self.clustering.clusters)))
        self.publish_clustered_points()

        self.cluster_filter.input_data(self.clustering.get_clusters())

        self.publish_clusters()

        self.position_predictor.get_predicted_position()
        self.beacon_library.transform_to_robot_ref(self.position_predictor)
        self.beacon_library.identify_clusters(self.cluster_filter.get_output(), self.position_predictor)
        # self.get_logger().info('---')
        # for beacon in self.beacon_library.beacons:
        #     self.get_logger().info("beacon {} --> {}".format(beacon.predicted_position, beacon.nearest_cluster.center if beacon.nearest_cluster else None))
        self.publish_predicted_beacons()

        # self.get_logger().info("position candidates pos : {}".format(self.beacon_library.compute_possible_positions()))
        # self.get_logger().info("position candidates ang : {}".format(self.beacon_library.diffs))
        self.get_logger().info("position estimate : {}".format(self.beacon_library.compute_position()))

        position = self.beacon_library.compute_position_and_angle()
        self.get_logger().info(str(position))
        if position:
            self.position_predictor.set_lidar_position(position)





    def process_odom(self, data:Odometry):
        self.position_predictor.process_odom_msg(data)



    def publish_filtered_points(self, points:list[Point]):
        cloud = PointCloud()
        cloud.header.frame_id = "laser_frame"
        cloud.channels = []

        for point in points:
            cloud.points.append(point.get_rclpy_point())

        self.filtered_points_publisher.publish(cloud)

    def publish_clustered_points(self):
        cloud = PointCloud()
        cloud.header.frame_id = "laser_frame"
        channel = ChannelFloat32()
        channel.name = "Cluster ID"
        cloud.channels = [channel]

        for point in self.clustering.points:
            cloud.channels[0].values.append(point.cluster_id if point.cluster_id != None else -1)
            cloud.points.append(point.get_rclpy_point())

        self.clustered_points_publisher.publish(cloud)

    def publish_clusters(self):
        cloud = PointCloud()
        cloud.header.frame_id = "laser_frame"
        channel = ChannelFloat32()
        channel.name = "Cluster ID"
        cloud.channels = [channel]

        for id, cluster in enumerate(self.cluster_filter.clusters):
            cloud.channels[0].values.append(id)
            cloud.points.append(cluster.center.get_rclpy_point())

        self.clusters_publisher.publish(cloud)

    def publish_predicted_beacons(self):
        cloud = PointCloud()
        cloud.header.frame_id = "laser_frame"
        channel = ChannelFloat32()
        channel.name = "Beacon ID"
        cloud.channels = [channel]

        for beacon in self.beacon_library.beacons:
            cloud.channels[0].values.append(beacon.id)
            cloud.points.append(beacon.predicted_position.get_rclpy_point())

        self.predicted_beacons_publisher.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


