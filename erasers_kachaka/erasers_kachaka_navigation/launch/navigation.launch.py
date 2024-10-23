#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from launch.actions import (
    IncludeLaunchDescription, 
    ExecuteProcess, 
    DeclareLaunchArgument, 
    GroupAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    param_file_path = os.path.join(
        get_package_share_directory('erasers_kachaka_navigation'),
        'cfg', 'navigation.yaml'
    )
    map_path = os.path.join(
        get_package_share_directory('erasers_kachaka_navigation'),
        'map',
        'map.yaml'
    )

    ## configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', default=param_file_path)
    slam_mode = LaunchConfiguration('slam_mode')
    
    ## arg
    declare_map_file = DeclareLaunchArgument(
        "map_file",
        default_value=map_path
    )
    declare_slam_mode = DeclareLaunchArgument(
        "slam_mode",
        default_value="false"
    )

    nav_launch_slam = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        get_package_share_directory("nav2_bringup"), '/launch/navigation_launch.py'
                    ]),
                    launch_arguments={
                        'params_file': params_file,
                        'use_sim_time': use_sim_time
                    }.items(),
                    condition=IfCondition(LaunchConfiguration('slam_mode'))
                )
    nav_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        get_package_share_directory("nav2_bringup"), '/launch/navigation_launch.py'
                    ]),
                    launch_arguments={
                        'params_file': params_file,
                        'use_sim_time': use_sim_time,
                        'map': LaunchConfiguration('map_file'),
                    }.items(),
                    condition=UnlessCondition(LaunchConfiguration('slam_mode'))
                )

    nav2_include = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel',dst='/er_kachaka/manual_control/cmd_vel'),
            SetRemap(src='/odom',dst='/kachaka/odometry/odometry'),
            SetRemap(src='/scan',dst='/kachaka/lidar/scan'),
            declare_map_file,
            declare_slam_mode,
            nav_launch_slam,
            nav_launch,
        ]
    )

    ld.add_action(nav2_include)

    return ld
