from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    ld = LaunchDescription()

    map_yaml_prefix = os.path.join(
        get_package_share_directory("erasers_kachaka_cartographer"),
        "map", "220-20241101.yaml"
    )

    map_server = Node(
        package="nav2_map_server",
        executable='map_server',
        name='map_server',
        output='screen',
        namespace="er_kachaka",
        parameters=[{'yaml_filename': map_yaml_prefix}]
    )

    ld.add_action(map_server)

    return ld
