#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    ld = LaunchDescription()

    ## Prefix
    slam_toolbox_prefix = get_package_share_directory("slam_toolbox")
    er_cartographer_prefix = get_package_share_directory("erasers_kachaka_cartographer")
    slam_toolbox_params_prefix = os.path.join(
        er_cartographer_prefix, "params", "slam_toolbox_params.yaml"
    )

    ## config
    slam_params_file = LaunchConfiguration("slam_params_file", default=slam_toolbox_params_prefix)

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                slam_toolbox_prefix, "/launch/",
                "online_async_launch.py"
            ]
        ),
        launch_arguments={"slam_params_file": slam_params_file}.items(),
    )

    ld.add_action(slam_toolbox_launch)

    return ld
