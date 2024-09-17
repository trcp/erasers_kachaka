#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    # map
    map2odom_pose = ["0", "0", "0", "0", "0", "0", "map", "odom"]

    # pkg prefix
    control_pkg_prefix = get_package_share_directory("erasers_kachaka_control")

    # include launch
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [control_pkg_prefix, "/launch/joy_teleop.launch.py"]
        ),
    )

    # include node
    service_tts = Node(
        package="erasers_kachaka_common",
        executable="service_tts",
    )

    emergency_manager = Node(
        package="erasers_kachaka_common",
        executable="emergency_manager",
    )

    rth_manager = Node(
        package="erasers_kachaka_common",
        executable="rth_manager",
    )

    docking_manager = Node(
        package="erasers_kachaka_common",
        executable="docking_manager",
    )

    battery_manager = Node(
        package="erasers_kachaka_common",
        executable="battery_manager",
    )

    sound_manager = Node(
        package="erasers_kachaka_common",
        executable="sound_manager",
    )

    map2odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=map2odom_pose,
    )

    # launch message
    startup_sound_command = ExecuteProcess(
        cmd=[
            'ros2 service call /er_kachaka/tts erasers_kachaka_interfaces/srv/Speaker "{text: erasersカチャカ、スタート}"'
        ],
        shell=True,
        output="both",
    )

    # add actions
    ld.add_action(teleop_launch)
    ld.add_action(service_tts)
    ld.add_action(emergency_manager)
    ld.add_action(rth_manager)
    ld.add_action(docking_manager)
    ld.add_action(battery_manager)
    ld.add_action(sound_manager)
    #ld.add_action(map2odom)
    ld.add_action(startup_sound_command)

    # include and execute
    return ld
