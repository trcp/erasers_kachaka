#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    param_file_path = os.path.join(
        get_package_share_directory('erasers_kachaka_navigation'),
        'cfg', 'navigation.yaml'
    )

    ## configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', default=param_file_path)

    nav_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        get_package_share_directory("nav2_bringup"), '/launch/navigation_launch.py'
                    ]),
                    launch_arguments={
                        'params_file': params_file,
                        'use_sim_time': use_sim_time
                    }.items(),
                )

    ld.add_action(nav_launch)

    return ld
