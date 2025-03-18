import copy
import os
import time
import pandas as pd
import rclpy
import yaml 
from rclpy.node import Node
from pathlib import Path
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
import numpy as np
from sklearn.cluster import DBSCAN
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from rclpy.qos import qos_profile_sensor_data

def intersections_cercles(x1:float, y1:float, r1:float, x2:float, y2:float, r2:float) -> list[list[float]]:
    d = np.sqrt((x2-x1)**2+(y2-y1)**2)  # distance entre les 2 points
    
    if d > (r1+r2) : return None
    if d < 0.0001 : return None

    l = (r1**2-r2**2+d**2)/(2*d)
    # h = np.sqrt(r1**2-l**2)
    h_squared = r1**2 - l**2
    if h_squared < 0:
        return None
    h = np.sqrt(h_squared)
    x = x1 + l/d*(x2-x1)
    dx = h/d*(y2-y1)
    y = y1 + l/d*(y2-y1)
    dy = h/d*(x2-x1)
    ax, ay = x+dx, y-dy
    bx, by = x-dx, y+dy

    return [[ax, ay],[bx, by]]


class Cluster:
    points:list[list[float]]=[]
    id:float
    center:list[float]=None

    def get_size(self) -> float :
        list_x = []
        list_y = []
        for point in self.points:
            list_x.append(point[0])
            list_y.append(point[1])
        x_min = min(list_x)
        x_max = max(list_x)
        y_min = min(list_y)
        y_max = max(list_y)
        return np.sqrt((x_max-x_min)**2+(y_max-y_min)**2)
    
    def get_distance(self, cluster:"Cluster") -> float:
        a = self.get_center()
        b = cluster.get_center()
        return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)

    def get_center(self) -> list[float]:
        if self.center == None:
            list_x = []
            list_y = []
            for point in self.points:
                list_x.append(point[0])
                list_y.append(point[1])
            self.center = [np.mean(list_x), np.mean(list_y)]

        return self.center
    
    def get_angle(self, A:"Cluster", B:"Cluster") -> float :
        a_O_B = np.arctan2(B.center[1]-self.center[1], B.center[0]-self.center[0])
        a_O_A = np.arctan2(A.center[1]-self.center[1], A.center[0]-self.center[0])

        return a_O_B - a_O_A
    
    def get_distance_robot(self) -> float :
        distance = np.sqrt(self.center[0]**2+self.center[1]**2)

        return distance
    
    def get_angle_robot(self) -> float :
        angle = np.arctan2(self.center[1], self.center[0])

        return angle

    def get_center_point_32(self) -> Point32:
        point = Point32()
        point.x, point.y = self.get_center()
        point.z = 0.0 #float

        return point



class Beacon:
    position:list[float]=[]

    def __init__(self, position:list[float]):
        self.position = position

    def get_distance(self, beacon:"Beacon") -> float:
        a = self.position
        b = beacon.position
        return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)
    
    def get_angle(self, A:"Beacon", B:"Beacon") -> float :
        a_O_B = np.arctan2(B.position[1]-self.position[1], B.position[0]-self.position[0])
        # angle entre le point et la balise B
        a_O_A = np.arctan2(A.position[1]-self.position[1], A.position[0]-self.position[0])
        # angle entre le point et la balise A

        return a_O_B - a_O_A
        # angle entre A et B


class FinalNode(Node):

    config_filtrage_lidar_min:float = 0.2
    config_filtrage_lidar_max:float = 4.0       # diagonale terrain : 3.6 m
    config_clustering_epsilon:float = 0.1       # on autorise au lidar de manquer 1 point
    # valeur initiale 0.1
    config_clustering_min_samples:int = 3       # nb de points min pour un cluster (augmenter à 4 si pas trop de faux-positifs)
    config_clustering_dist_max:float = 0.16     # taille max d'un cluster pouvant être considéré comme une balise
    # config_beacon_0:Beacon = Beacon([1.0, 1.5])
    # config_beacon_1:Beacon = Beacon([-1.0, 1.5])
    # config_beacon_2:Beacon = Beacon([0.0, -1.5])

    config_beacons = [Beacon([1.0, 1.5]), Beacon([-1.0, 1.5]), Beacon([0.0, -1.5])]

    config_beacon_offset:float = 0.05*(2/3)   # remplacer 2/3 par 3/4 
    config_identification_error_threshold:float = 0.6  # somme des erreurs de distances au carré

    # map_yaml_path = Path("/home/charlotte/maps/map_arene.yaml")
    
    # map_yaml_path = Path("/home/charlotte/maps/map_arene.yaml")
    
    # with open(map_yaml_path, 'r') as file:
    #     map_info = yaml.safe_load(file)

    # resolution = map_info["resolution"]  
    # origin = map_info["origin"] 
    # dimensions = map_info["dimensions"] 
    # beacons = map_info["beacons"] 
    # # config_beacons:list[Beacon] = [Beacon(beacon["position"]) for beacon in beacons]
    # config_beacons:list[Beacon] = []
    # for beacon in beacons:
    #     config_beacons.append(Beacon(beacon["position"]))

    def __init__(self):
        super().__init__("final_node")

        # paramètres du tableau 
        self.table_size = 100  # nombre de lignes du tableau
        self.position_history = []  # stocke les dernières positions
        self.filename = "positions.csv"

        # qos_profile = QoSProfile(
        # reliability=QoSReliabilityPolicy.RELIABLE,
        # history=QoSHistoryPolicy.KEEP_LAST,
        # depth=10,
        # durability=QoSDurabilityPolicy.VOLATILE,
        # liveliness=QoSLivelinessPolicy.AUTOMATIC
        # )

        qos_profile = qos_profile_sensor_data

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.process,
            qos_profile)

        self.clusters_publisher = self.create_publisher(
            PointCloud,
            '/clusters_tries',
            10
        )

        self.beacons_publisher = self.create_publisher(
            PointCloud,
            '/identified_beacons',
            10
        )

        self.points_publisher = self.create_publisher(
            PointCloud,
            '/points',
            10
        )
        
    def process(self, scan:LaserScan):

        self.get_logger().info("----")

        distances, angles = self.filtrage_lidar(scan)
        positions = FinalNode.polar_to_cartesian(angles, distances)   # FinalNode car méthode statique donc pas de self
        clusters = self.clustering(positions)

        if len(clusters) < 3:
            self.get_logger().info("not enough clusters")
            return

        
        clusters_tries = self.filter_clusters(clusters)

        self.publish_clusters(clusters_tries)

        self.get_logger().info(f"Clusters détectés : {len(clusters)}")
        self.get_logger().info(f"Clusters après filtrage : {len(clusters_tries)}")

        # for i, cluster in enumerate(clusters_tries):
        #     self.get_logger().info(f"Cluster {i}: position {cluster.get_center()} taille {cluster.get_size()}")

        if len(clusters_tries) < 3:
            self.get_logger().info("not enough identified clusters")
            return
        
        identified_beacons, identification_error = self.identify_beacons(clusters_tries)

        if identification_error > self.config_identification_error_threshold:
            self.get_logger().info("identification error threshold triggered")
            return

        self.publish_beacons(identified_beacons)
        
        x, y = self.triangulation(identified_beacons)

        if x == None or y == None :
            self.get_logger().info("triangulation failed")
            return

        theta = self.calcul_rotation(identified_beacons, x, y)

        self.get_logger().info("estimated position : x = {:.03f}, y = {:.03f}, theta = {:.01f},".format(x, y ,theta*180.0/np.pi))

        # self.update_position_history(x, y, theta)
        # self.save_to_csv()


    def filtrage_lidar(self, scan:LaserScan) -> tuple[list[float], list[float]]:
        distances = []
        angles = []
        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + scan.angle_increment*index
            if self.config_filtrage_lidar_min < distance < self.config_filtrage_lidar_max :
                distances.append(distance)
                angles.append(angle)
        
        return (distances, angles)
    
    @staticmethod
    def polar_to_cartesian(angles:list[float], distances:list[float]) -> list[list[float]]:
        positions = []
        for index in range(len(angles)):
            position_x = distances[index]*np.cos(angles[index])
            position_y = distances[index]*np.sin(angles[index])
            positions.append([position_x, position_y])

        return positions 
    
    def clustering(self, positions:list[list[float]]) -> list[Cluster]:
        db = DBSCAN(eps=self.config_clustering_epsilon, min_samples=self.config_clustering_min_samples).fit(positions)
        clusters_dict = {}
        for index, id in enumerate(db.labels_):
            if id<0:
                continue
            if id not in clusters_dict.keys():
                clusters_dict[int(id)] = []
            clusters_dict[int(id)].append(positions[index])
            # dictionnaire de clusters à transformer en liste

        cluster_list : list[Cluster] = []
        for cluster_points in clusters_dict.values():
            cluster_list.append(Cluster())
            cluster_list[-1].points = cluster_points

        self.publish_points(positions, db.labels_)

        return cluster_list
    
    def filter_clusters(self, a_trier:list[Cluster]) -> list[Cluster]:
        clusters = []
        for cluster in a_trier:
            if cluster.get_size() < self.config_clustering_dist_max:
                clusters.append(cluster)
        
        return clusters
    
    def identify_beacons(self, clusters:list[Cluster]) -> tuple[list[Cluster], float]:
        # renvoie une liste de clusters triée
        # trouver les 3 balises et voir celle qui est le plus loin
        d_0_1 = self.config_beacons[0].get_distance(self.config_beacons[1]) - self.config_beacon_offset
        d_1_2 = self.config_beacons[1].get_distance(self.config_beacons[2]) - self.config_beacon_offset
        d_2_0 = self.config_beacons[2].get_distance(self.config_beacons[0]) - self.config_beacon_offset
        best_error = 999.9
        best_guess = []
        a_0_1_2 = self.config_beacons[1].get_angle(
            self.config_beacons[0], self.config_beacons[2])
    
        for iA, A in enumerate(clusters):
            for iB, B in enumerate(clusters):
                if iA == iB:
                    continue
                for iC, C in enumerate(clusters):
                    if iC in [iA, iB]:
                        continue
                    d_A_B = A.get_distance(B)
                    d_B_C = B.get_distance(C)
                    d_C_A = C.get_distance(A)

                    # self.get_logger().info(f"Distances réelles entre balises : d_0_1={d_0_1}, d_1_2={d_1_2}, d_2_0={d_2_0}")
                    # self.get_logger().info(f"Distances des clusters identifiés : d_A_B={d_A_B}, d_B_C={d_B_C}, d_C_A={d_C_A}")

                    error = (d_A_B - d_0_1)**2
                    error += (d_B_C - d_1_2)**2
                    error += (d_C_A - d_2_0)**2

                    if error < best_error:
                        best_error = error
                        best_guess = [A, B, C]

        if len(best_guess) < 3:
            self.get_logger().info("Beacon identification failed")
            return [], 999.9
        
        a_A_B_C = best_guess[1].get_angle(best_guess[0], best_guess[2])
        a_B_A_C = best_guess[0].get_angle(best_guess[1], best_guess[2])

        # self.get_logger().info(f"Angles entre clusters identifiés : {a_A_B_C} et {a_B_A_C}, attendu : {a_0_1_2}")

        delta_A_B_C = abs(np.remainder(a_A_B_C - a_0_1_2, np.pi*2))     # modulo 2pi
        delta_B_A_C = abs(np.remainder(a_B_A_C - a_0_1_2, np.pi*2))

        if delta_A_B_C < delta_B_A_C :
            return best_guess, best_error
        
        return [best_guess[1], best_guess[0], best_guess[2]], best_error
    
    def triangulation(self, identified_beacons:list[Cluster]):
        combinaisons = [[0,1], [1,2], [2,0]]    # combinaisons de beacons
        beacons = [self.config_beacons[0], self.config_beacons[1], self.config_beacons[2]]
        liste_couples_intersections = []
        for id1, id2 in combinaisons :
            x1, y1 = beacons[id1].position
            x2, y2 = beacons[id2].position
            r1 = identified_beacons[id1].get_distance_robot() + self.config_beacon_offset
            r2 = identified_beacons[id2].get_distance_robot() + self.config_beacon_offset
            intersections = intersections_cercles(x1, y1, r1, x2, y2, r2)
            if not intersections:
                continue
            liste_couples_intersections.append(intersections)
        if len(liste_couples_intersections) != 3:
            return None, None
        min_error = 999.9
        best_guess = []
        for id0 in range(2):
            p0 = liste_couples_intersections[0][id0]
            for id1 in range(2):
                p1 = liste_couples_intersections[1][id1]
                for id2 in range(2):
                    p2 = liste_couples_intersections[2][id2]
                    error = (p2[0]-p1[0])**2+(p2[1]-p1[1])**2+(p2[0]-p0[0])**2+(p2[1]-p0[1])**2+(p1[0]-p0[0])**2+(p1[1]-p0[1])**2
                    if error < min_error:
                        min_error = error
                        best_guess = [p0, p1, p2]
        if not best_guess:
            return None, None
        position_moy = np.mean(best_guess, axis=0)
        if not isinstance(position_moy, (list, np.ndarray)) or len(position_moy) < 2:
            return None, None
        return position_moy[0], position_moy[1]
    
    def calcul_rotation(self, identified_beacons:list[Cluster], x:float, y:float) -> float:
        beacons = [self.config_beacons[0], self.config_beacons[1], self.config_beacons[2]]
        estimated_rotations = []
        for id in range(3):
            cluster = identified_beacons[id]
            beacon = beacons[id]
            angle_cluster = cluster.get_angle_robot()
            angle_beacon = np.arctan2(beacon.position[1]-y, beacon.position[0]-x)
            diff_angles = angle_beacon - angle_cluster  # autre sens ?
            estimated_rotations.append(diff_angles)
        c = np.mean(np.cos(estimated_rotations))
        s = np.mean(np.sin(estimated_rotations))
        rotation_moy = np.arctan2(s, c)

        return rotation_moy
    
    def update_position_history(self, x, y, theta):
        current_time = time.time()
        if len(self.position_history) >= self.table_size:
            self.position_history.pop(0)  # supprime la plus ancienne valeur
        self.position_history.append([current_time, x, y, theta])  # ajoute la nouvelle valeur

    def save_to_csv(self, directory="/home/charlotte/Documents/Polytech/Robotech"):
        if not os.path.exists(directory):
            os.makedirs(directory)

        filepath = os.path.join(directory, "positions.csv")

        df = pd.DataFrame(self.position_history, columns=["Timestamp", "x (m)", "y (m)", "θ (rad)"])

        df["θ (deg)"] = df["θ (rad)"] * 180.0 / np.pi  # ajout d'une colonne en degrés

        df["Temps écoulé (s)"] = df["Timestamp"].diff().fillna(0)

        df.drop(columns=["θ (rad)", "Timestamp"], inplace=True)

        df.to_csv(filepath, index=False)

    def publish_clusters(self, clusters:list[Cluster]):
        msg = PointCloud()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.points = []
        msg.channels = [ChannelFloat32()]
        msg.channels[0].name = "cluster_id"
        msg.channels[0].values = []
        for id, cluster in enumerate(clusters) :
            msg.points.append(cluster.get_center_point_32())
            msg.channels[0].values.append(id)

        self.clusters_publisher.publish(msg)

    def publish_beacons(self, clusters:list[Cluster]):
        msg = PointCloud()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.points = []
        msg.channels = [ChannelFloat32()]
        msg.channels[0].name = "beacon_id"
        msg.channels[0].values = []
        for id, cluster in enumerate(clusters) :
            msg.points.append(cluster.get_center_point_32())
            msg.channels[0].values.append(id)

        self.beacons_publisher.publish(msg)

    def publish_points(self, points:list[list[float]], labels:list[int]):
        point_32 = []
        for i in range (len(points)):
            point_32.append(Point32())
            point_32[i].x, point_32[i].y = points[i]
            point_32[i].z = 0.0
        msg = PointCloud()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.points = point_32
        msg.channels = [ChannelFloat32()]
        msg.channels[0].name = "beacon_id"
        msg.channels[0].values = labels

        self.points_publisher.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = FinalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()