#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os

KACHAKA_NAME = os.environ.get('KACHAKA_NAME')

BRINGUP_MSG = "Kachaka スタート！"


def generate_launch_description():
    ld = LaunchDescription()


    prefix_erk_teleop = get_package_share_directory("erasers_kachaka_teleop")
    prefix_erk_description = get_package_share_directory("erasers_kachaka_description")
    prefix_rviz = os.path.join(
        get_package_share_directory("erasers_kachaka_bringup"),
        "rviz", "erasers_kachaka.rviz"
    )


    # config
    config_use_rviz = LaunchConfiguration("use_rviz")
    config_use_shelf = LaunchConfiguration("use_shelf")


    # declare arguments
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="True",
        description="Rviz2 を起動します"
    )
    declare_use_shelf = DeclareLaunchArgument(
        "use_shelf", default_value="True",
        description="シェルフが搭載されているなら True にしてください。"
    )

    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_shelf)


    # NODES
    node_kachaka_speak_subscriber = Node(
        package="erasers_kachaka_common",
        executable="kachaka_speak_subscriber",
        namespace=KACHAKA_NAME
    )
    node_emergency_manager = Node(
        package="erasers_kachaka_common",
        executable="emergency_manager",
        output="screen",
        namespace=KACHAKA_NAME
    )
    node_lidar_observer = Node(
        package="erasers_kachaka_common",
        executable="lidar_observer",
        output="screen",
        namespace=KACHAKA_NAME
    )
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", prefix_rviz],
        condition=IfCondition(config_use_rviz)
    )
    node_mapprovider = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="own_log",
    )

    ld.add_action(node_kachaka_speak_subscriber)
    ld.add_action(node_emergency_manager)
    ld.add_action(node_lidar_observer)
    ld.add_action(node_rviz)
    ld.add_action(node_mapprovider)


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
    launch_description = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            prefix_erk_description,
            "/launch/erasers_kachaka_description.launch"
        ])
    )
    launch_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            prefix_erk_teleop,
            "/launch/teleop.launch.py"
        ])
    )

    ld.add_action(launch_description)
    ld.add_action(launch_teleop)


    return ld
