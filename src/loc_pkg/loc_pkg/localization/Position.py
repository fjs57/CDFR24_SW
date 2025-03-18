import copy
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

class Position:
    x : float
    y : float
    theta : float

    def __init__(self, x:float=None, y:float=None, theta:float=0):
        self.x = x
        self.y = y
        self.theta = theta

    def __sub__(self, b:"Position"):
        new = Position()
        new.x = self.x - b.x
        new.y = self.y - b.y
        new.theta = self.theta - b.theta
        return new
    
    def __add__(self, b:"Position"):
        new = Position()
        new.x = self.x + b.x
        new.y = self.y + b.y
        new.theta = self.theta + b.theta
        return new
    
    def rotate_around_point(self, center:"Position", rot:float):
        self.theta += rot

        a = np.array([[self.x],[self.y]])
        b = np.array([[center.x],[center.y]])
        r = np.array([[np.cos(rot), -np.sin(rot)],[np.sin(rot), np.cos(rot)]])

        d = a-b
        d = r.dot(d)

        s = b + d
        self.x = s[0, 0]
        self.y = s[1, 0]
    
    def copy(self)->'Position':
        return copy.deepcopy(self)
    
    def from_odometry_msg(self, msg:Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.theta = np.arctan2(2*qz*qw, 1-2*(qz**2))

    def get_pose(self)->Pose:
        qz = np.sin(self.theta/2)
        qw = np.cos(self.theta/2)
        new = Pose()
        new.position.x = self.x
        new.position.y = self.y
        new.position.z = 0.0
        new.orientation.x = 0.0
        new.orientation.y = 0.0
        new.orientation.z = qz
        new.orientation.w = qw
        return new
    
    @staticmethod
    def create_from_odometry(msg:Odometry):
        new = Position()
        new.from_odometry_msg(msg)
        return new
    
    def __str__(self) -> str:
        return "pos:[{},{}] rot:{}".format(self.x, self.y, self.theta)