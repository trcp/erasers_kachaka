#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetParameter

import os

from ament_index_python.packages import get_package_share_directory

MAP = os.path.join(
    get_package_share_directory("erasers_kachaka_cartographer"),
    "map", "220-20241101.yaml"
)

def generate_launch_description():
    ld = LaunchDescription()

    params_file = LaunchConfiguration("params_file")
    map_yaml_file = LaunchConfiguration("map", default=MAP)
    use_sim_time = LaunchConfiguration("use_sim_time", default=False)
    log = LaunchConfiguration("log", default="screen")

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=[
            TextSubstitution(
                text=os.path.join(get_package_share_directory("emcl2"), "config", "")
            ),
            TextSubstitution(text="emcl2.param.yaml"),
        ],
        description="emcl2 param file path",
    )

    lifecycle_nodes = ["map_server"]

    launch_node = GroupAction(
        actions=[
            SetParameter("use_sim_time", use_sim_time),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                namespace="/er_kachaka/navigation",
                parameters=[{"yaml_filename": map_yaml_file}],
                output=log,
            ),
            Node(
                name="emcl2",
                package="emcl2",
                executable="emcl2_node",
                parameters=[params_file],
                namespace="/er_kachaka/navigation",
                remappings=[("scan", "/er_kachaka/lidar/scan")],
                output="own_log",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                namespace="/er_kachaka/navigation",
                output=log,
                parameters=[{"autostart": True}, {"node_names": lifecycle_nodes}],
            ),
        ]
    )

    ld.add_action(declare_params_file)

    ld.add_action(launch_node)

    return ld