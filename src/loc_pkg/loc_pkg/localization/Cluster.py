import numpy as np
from .Point import Point
import copy

class Cluster:
    points : list[Point] = []
    center : Point = None
    label : int

    def insert_point(self, point:Point):
        assert(point)
        assert(isinstance(point, Point))
        p = copy.deepcopy(point)
        self.points.append(p)

    def compute_cluster_position(self):
        assert(len(self.points)!=0)
        if not self.center:
            self.center = Point()
        [p.compute_cartesian for p in self.points]
        self.center.pos_x = np.mean([p.pos_x for p in self.points])
        self.center.pos_y = np.mean([p.pos_y for p in self.points])
        self.center.compute_polar()

    def get_size(self):
        x = [p.pos_x for p in self.points]
        y = [p.pos_y for p in self.points]
        dx = max(x)-min(x)
        dy = max(y)-min(y)
        return np.sqrt(dx**2+dy**2)
