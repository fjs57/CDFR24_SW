
import time
import numpy as np

from nav_msgs.msg import Odometry

from .Position import Position
from .Point import Point

class PositionPredictor:
    position_from_lidar : Position
    position_from_odom_initial : Position = None
    position_from_odom_current : Position = None
    current_prediction : Position

    last_lidar_update : float
    odom_distance_after_last_lidar_update : float = 0.0
    odom_turn_after_last_lidar_update : float = 0.0

    config_lidar_pos_stddev : float = 0.0
    config_lidar_ang_stddev : float = 0.0
    config_odom_distance_error : float = 0.0 # error dependant on distance for wheel odometry
    config_odom_turn_error : float = 0.0 # error dependant on rotations for wheel odometry

    def set_config(self, 
                   lidar_pos_stddev:float=0.0, lidar_ang_stddev:float=0.0, 
                   odom_dist_error:float=0.0, odom_turn_error:float=0.0):
        self.config_lidar_ang_stddev = lidar_ang_stddev
        self.config_lidar_pos_stddev = lidar_pos_stddev
        self.config_odom_distance_error = odom_dist_error
        self.config_odom_turn_error = odom_turn_error

    def set_initial_position(self, x:float, y:float, theta:float):
        self.position_from_odom_current = Position(x, y, theta)
        self.set_lidar_position(Position(x, y, theta), force=True)

    def set_lidar_position(self, pos:Position, force:bool=False):
        self.position_from_lidar = pos
        self.odom_distance_after_last_lidar_update = 0.0
        self.odom_turn_after_last_lidar_update = 0.0
        self.last_lidar_update = time.time()
        if self.position_from_odom_current:
            self.position_from_odom_initial = self.position_from_odom_current.copy()

    def set_odom_position(self, pos:Position):
        if self.position_from_odom_current == None:
            self.odom_distance_after_last_lidar_update = 0.0
            self.odom_turn_after_last_lidar_update = 0.0
        else:
            diff = pos - self.position_from_odom_current
            self.odom_distance_after_last_lidar_update += np.sqrt(diff.x**2+diff.y**2)
            self.odom_turn_after_last_lidar_update += np.abs(diff.theta)

        self.position_from_odom_current = pos

    def get_predicted_position(self)->Position:

        if self.position_from_odom_initial == None:
            if self.position_from_lidar == None:
                return None
            return self.position_from_lidar

        new = Position()
        i = self.position_from_odom_initial.copy()
        c = self.position_from_odom_current.copy()
        r = self.position_from_lidar.copy()

        dt = i.theta - r.theta

        i.rotate_around_point(r, dt)
        c.rotate_around_point(r, dt)

        new = r + c - i
        self.current_prediction = new
        return new.copy()
    
    def get_prediction_error(self):
        pos_error = self.config_lidar_pos_stddev + self.odom_distance_after_last_lidar_update * self.config_odom_distance_error
        ang_error = self.config_lidar_ang_stddev + self.odom_turn_after_last_lidar_update * self.config_odom_turn_error
        return (pos_error, ang_error)
    
    def process_odom_msg(self, msg:Odometry):
        self.set_odom_position(Position.create_from_odometry(msg))

    def transform_from_odom_to_robot_ref(self, point:Point):
        self.get_predicted_position()
        point.translate(-self.current_prediction.x, -self.current_prediction.y)
        point.compute_polar()
        point.rotate(-self.current_prediction.theta)
        return point







