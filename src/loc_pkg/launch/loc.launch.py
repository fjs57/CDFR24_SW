import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pkg_name = "loc_pkg"

    loc_param_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'loc_cfg.yaml')
    beacon_param_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'beacons.yaml')
    init_param_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'init.yaml')
    
    loc_node = Node(
        package=pkg_name,
        executable="localization_node",
        parameters= [loc_param_file, beacon_param_file, init_param_file]
    )

    return LaunchDescription([
        loc_node
    ])