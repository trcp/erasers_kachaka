#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os

KACHAKA_NAME = os.environ.get('KACHAKA_NAME')


def generate_launch_description():
    ld = LaunchDescription()

    prefix_erk_teleop = get_package_share_directory("erasers_kachaka_teleop")

    bringup_docker = ExecuteProcess(
        cmd=[[
            "docker compose",
            " -f %s/docker/docker-compose.yaml"%os.environ.get('KACHAKA_ERK_PATH'),
            " up kachaka"
        ]],
        shell=True
    )

    ld.add_action(bringup_docker)


    launch_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            prefix_erk_teleop,
            "/launch/teleop.launch.py"
        ])
    )

    ld.add_action(launch_teleop)


    return ld
