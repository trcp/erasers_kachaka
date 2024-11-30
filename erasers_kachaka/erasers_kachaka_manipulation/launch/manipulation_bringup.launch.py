#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    description = os.path.join(
        get_package_share_directory("erasers_kachaka_manipulation"),
        "urdf",
        "manipulation_description.urdf.xacro"
    )
    
    manipulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("nakalab_crane_x7_bringup"),
                "launch",
                "bringup.launch.py"
            )
        ]),
        launch_arguments={
            "description":description,
            "use_d435":"true"
        }.items()
    )

    ld.add_action(manipulation_launch)

    return ld