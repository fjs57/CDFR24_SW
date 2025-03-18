import numpy as np
from geometry_msgs.msg import Point32

class Point:
    pos_x : float = None
    pos_y : float = None
    pos_r : float = None
    pos_th : float = None
    cluster_id : int = None

    def compute_cartesian(self):
        self.pos_x = np.cos(self.pos_th) * self.pos_r
        self.pos_y = np.sin(self.pos_th) * self.pos_r

    def compute_polar(self):
        self.pos_r = np.sqrt(self.pos_x**2+self.pos_y**2)
        self.pos_th = np.arctan2(self.pos_y, self.pos_x)

    def get_cartesian_list(self):
        if self.pos_x == None or self.pos_y==None:
            self.compute_cartesian()
        return [self.pos_x, self.pos_y]
    
    def get_rclpy_point(self)->Point32:
        p = Point32()
        self.compute_cartesian()
        p.z = 0.0
        p.x = self.pos_x
        p.y = self.pos_y
        return p

    def get_distance_and_angle_from_reference(self, reference:"Point"):
        self.compute_cartesian()
        reference.compute_cartesian()
        dx = self.pos_x - reference.pos_x
        dy = self.pos_y - reference.pos_y
        return np.sqrt(dx**2+dy**2), np.arctan2(dy, dx)
    
    def get_distance_and_angle_to_point(self, point:"Point"):
        # self.compute_cartesian()
        # point.compute_cartesian()
        dx = point.pos_x - self.pos_x
        dy = point.pos_y - self.pos_y
        return np.sqrt(dx**2+dy**2), np.arctan2(dy, dx)
    
    def translate(self, dx:float, dy:float):
        if self.pos_x == None or self.pos_y == None:
            self.compute_cartesian()
        self.pos_x += dx
        self.pos_y += dy
        self.compute_polar()

    def rotate(self, rot:float):
        self.compute_polar()
        self.pos_th += rot
        self.compute_cartesian()

    def offset_range(self, offset:float):
        if self.pos_r == None or self.pos_th == None:
            self.compute_polar()
        self.pos_r+=offset
        self.compute_cartesian()
    
    def __eq__(self, value:"Point"):
        return self.pos_x == value.pos_x and self.pos_y == value.pos_y and self.pos_r == value.pos_r and self.pos_th == value.pos_th
    
    def __str__(self):
        s = "Point =>"
        if self.pos_x!=None and self.pos_y!=None:
            s+= " xy:[{:.3f},{:.3f}]".format(self.pos_x, self.pos_y)
        if self.pos_r!=None and self.pos_th!=None:
            s+= " pol:[{:.3f},{:.3f}rad]".format(self.pos_r, self.pos_th)
        if self.cluster_id != None:
            s+= " cluster_id:{}".format(self.cluster_id)
        return s