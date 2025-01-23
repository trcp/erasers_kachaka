#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os

KACHAKA_NAME = os.environ.get('KACHAKA_NAME')

BRINGUP_MSG = "Kachaka スタート！"


def generate_launch_description():
    ld = LaunchDescription()


    prefix_erk_teleop = get_package_share_directory("erasers_kachaka_teleop")


    # NODES
    node_kachaka_speak_subscriber = Node(
        package="erasers_kachaka_common",
        executable="kachaka_speak_subscriber",
        namespace=KACHAKA_NAME
    )

    ld.add_action(node_kachaka_speak_subscriber)


    # PROCESS
    bringup_docker = ExecuteProcess(
        cmd=[[
            "docker compose",
            " -f %s/docker/docker-compose.yaml"%os.environ.get('KACHAKA_ERK_PATH'),
            " up kachaka"
        ]],
        shell=True
    )
    bringup_msg = RegisterEventHandler(
        OnProcessStart(
            target_action=node_kachaka_speak_subscriber,
            on_start=[
                ExecuteProcess(
                    cmd=[[
                        "ros2 topic pub --once",
                        " /%s/kachaka_speak"%KACHAKA_NAME,
                        " std_msgs/msg/String",
                        " \"{data: %s}\""%BRINGUP_MSG
                    ]],
                    shell=True
                )
            ]
        )
    )


    ld.add_action(bringup_docker)
    ld.add_action(bringup_msg)


    # LAUNCHERS
    launch_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            prefix_erk_teleop,
            "/launch/teleop.launch.py"
        ])
    )

    ld.add_action(launch_teleop)


    return ld
