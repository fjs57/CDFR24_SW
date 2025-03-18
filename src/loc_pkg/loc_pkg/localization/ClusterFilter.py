from .Cluster import Cluster

import copy

class ClusterFilter:

    config_max_point_count : int = None
    config_max_size : float = None
    clusters : list[Cluster]

    def set_config(self, max_point_count:int, max_size:float):
        self.config_max_point_count = max_point_count
        self.config_max_size = max_size

    def input_data(self, clusters:list[Cluster]):
        assert(clusters)
        assert(isinstance(clusters, list))
        assert(isinstance(clusters[0], Cluster))
        self.clusters = copy.deepcopy(clusters)

    def filter_by_point_count(self):
        assert(self.config_max_point_count)
        for c in self.clusters:
            if len(c.points) > self.config_max_point_count:
                self.clusters.remove(c)

    def filter_by_size(self):
        assert(self.config_max_size)
        for c in self.clusters:
            if c.get_size() > self.config_max_size:
                self.clusters.remove(c)

    def get_output(self):
        return self.clusters
