import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():

    package_name='robot_pkg'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name),'launch','rsp.launch.py')]
        ),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # joystick = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','joystick.launch.py'
    #             )]), launch_arguments={'use_sim_time': 'true'}.items()
    # )

    # twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    # twist_mux = Node(
    #         package="twist_mux",
    #         executable="twist_mux",
    #         parameters=[twist_mux_params, {'use_sim_time': True}],
    #         remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    #     )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
        ),
        launch_arguments={
            'world': "/home/francois/code/CDFR24_SW_dev/src/robot_pkg/world/odom_world.sdf",
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description','-entity', 'robot'],
        output='screen'
    )


    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    lidar_localization_node = Node(
        package='test_odom_pkg',
        executable='clustering_process'
    )

    robot_localization_file_path = os.path.join(get_package_share_directory("test_odom_pkg"),'config','ekf.yaml')
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, {'use_sim_time': True}]
    )



    # Launch them all!
    return LaunchDescription([
        rsp,
        # joystick,
        # twist_mux,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        # lidar_localization_node,
        start_robot_localization_cmd
    ])
