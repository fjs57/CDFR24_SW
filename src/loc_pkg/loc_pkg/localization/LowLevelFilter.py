from sensor_msgs.msg import LaserScan
from .Point import Point

class LowLevelFilter:

    points : list[Point]
    config_min_range : float = None
    config_max_range : float = None

    def set_config(self, min_range:float, max_range:float):
        self.config_min_range = min_range
        self.config_max_range = max_range

    def process_data(self, data:LaserScan)->list[Point]:
        assert(self.config_max_range)
        assert(self.config_min_range)
        assert(isinstance(data, LaserScan))
        start_angle = data.angle_min
        angle_increment = data.angle_increment
        samples = data.ranges
        self.points = []
        for index, range in enumerate(samples):
            angle = start_angle + angle_increment * index
            if not(self.config_min_range < range < self.config_max_range):
                continue
            point = Point()
            point.pos_r = range
            point.pos_th = angle
            self.points.append(point)
        return self.points
