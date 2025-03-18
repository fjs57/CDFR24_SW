import copy
from sklearn.cluster import DBSCAN
from .Point import Point
from .Cluster import Cluster

class Clustering:

    config_epsilon : float
    config_min_cluster_size : int
    points : list[Point] = None
    clusters : list[Cluster] = None
    

    def __init__(self, epsilon:float=None, min_cluster_size:int=None):
        self.set_config(epsilon, min_cluster_size)

    def set_config(self, epsilon:float, min_cluster_size:int):
        self.config_epsilon = epsilon
        self.config_min_cluster_size = min_cluster_size

    def set_input(self, points:list[Point]):
        assert(points)
        self.points = copy.deepcopy(points)

    def execute_clustering(self):
        assert(self.points)
        scan = [p.get_cartesian_list() for p in self.points]
        db = DBSCAN(eps=self.config_epsilon, min_samples=self.config_min_cluster_size).fit(scan)
        for index, cluster_id in enumerate(db.labels_):
            self.points[index].cluster_id = cluster_id if cluster_id!=-1 else None

    def create_clusters(self):
        clusters_dict = dict()
        for p in self.points:
            if p.cluster_id == None:
                continue
            if p.cluster_id not in clusters_dict.keys():
                clusters_dict[p.cluster_id] = []
            clusters_dict[p.cluster_id].append(p)
            
        self.clusters = []
        for pts in list(clusters_dict.values()):
            cluster = Cluster()
            cluster.points = pts
            cluster.compute_cluster_position()
            self.clusters.append(copy.deepcopy(cluster))
    
    def get_clusters(self)->list[Cluster]:
        assert(self.clusters)
        assert(isinstance(self.clusters, list))
        return copy.deepcopy(self.clusters)
        

