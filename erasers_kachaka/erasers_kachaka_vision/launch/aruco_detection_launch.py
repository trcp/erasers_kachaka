#!/usr/bin/env python3
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

KACHAKA_NAME = os.environ.get('KACHAKA_NAME')

def generate_launch_description():
    ld = LaunchDescription()

    default_params_file = os.path.join(
        get_package_share_directory('erasers_kachaka_vision'),
        'params/param_aruco_detection.yaml'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params_file,
        description='Aruco マーカーパラメータファイルを指定'
    )

    node_marker_publisher = Node(
        package='ros2_aruco',
        executable='aruco_node',
        parameters=[LaunchConfiguration('params_file')],
        emulate_tty=True,
        namespace=KACHAKA_NAME
    )

    ld.add_action(declare_params_file)
    ld.add_action(node_marker_publisher)

    return ld
