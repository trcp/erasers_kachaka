#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory

import os

NAMESPACE = "er_kachaka"

def generate_launch_description():
    ld = LaunchDescription()

    # launch configurations
    LaunchConfiguration("provide_map")
    show_rviz = LaunchConfiguration("show_rviz")
    use_emc = LaunchConfiguration("use_emc")
    use_xtion = LaunchConfiguration("use_xtion")
    ns = LaunchConfiguration("namespace", default=NAMESPACE)

    # map
    map2odom_pose = ["0", "0", "0", "0", "0", "0", "map", "odom"]
    odom2foot_pose = ["0", "0", "0", "0", "0", "0", "odom", "base_footprint"]

    # pkg prefix
    control_pkg_prefix = get_package_share_directory("erasers_kachaka_control")
    common_pkg_prefix = get_package_share_directory("erasers_kachaka_common")
    vision_pkg_prefix = get_package_share_directory("erasers_kachaka_vision")
    rviz_name = "erasers_kachaka.rviz"
    rviz_prefix = os.path.join(common_pkg_prefix, "config", rviz_name)

    # declare arguments
    declare_provide_map = DeclareLaunchArgument("provide_map",
                                                default_value="true",
                                                description="Whether to deliver a map frame from the host")
    declare_show_rviz = DeclareLaunchArgument("show_rviz",
                                                default_value="true",
                                                description="Whether to display Rviz2")
    declare_use_emc = DeclareLaunchArgument("use_emc",
                                                default_value="true",
                                                description="use emergency button")
    declare_use_xtion = DeclareLaunchArgument("use_xtion",
                                                default_value="false",
                                                description="use Xtion camera")

    # include launch
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [control_pkg_prefix, "/launch/joy_teleop.launch.py"]
        ),
        launch_arguments={"use_emc": use_emc,
                          "_namespace": ns}.items(),
    )

    xtion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [vision_pkg_prefix, "/launch/xtion.launch.py"]
        ),
        condition=IfCondition(use_xtion),
        launch_arguments={"_namespace": ns}.items(),
    )

    # include node
    service_tts = Node(
        package="erasers_kachaka_common",
        executable="service_tts",
        namespace=ns
    )

    emergency_manager = Node(
        package="erasers_kachaka_common",
        executable="emergency_manager",
        namespace=ns
    )

    rth_manager = Node(
        package="erasers_kachaka_common",
        executable="rth_manager",
        namespace=ns
    )

    docking_manager = Node(
        package="erasers_kachaka_common",
        executable="docking_manager",
        namespace=ns
    )

    battery_manager = Node(
        package="erasers_kachaka_common",
        executable="battery_manager",
        namespace=ns
    )

    sound_manager = Node(
        package="erasers_kachaka_common",
        executable="sound_manager",
        namespace=ns
    )

    map2odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=map2odom_pose,
        condition=IfCondition(LaunchConfiguration('provide_map')),
    )

    odom2foot = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=odom2foot_pose,
        condition=IfCondition(LaunchConfiguration('provide_map')),
    )

    rviz = Node(
        package = "rviz2",
        executable = "rviz2",
        condition=IfCondition(show_rviz),
        arguments=["-d", rviz_prefix]
    )

    # launch message
    startup_sound_command = ExecuteProcess(
        cmd=[
            'ros2 service call /%s/tts erasers_kachaka_interfaces/srv/Speaker "{text: erasersカチャカ、スタート}"'%NAMESPACE
        ],
        shell=True,
        output="own_log",
    )

    # add actions
    ld.add_action(declare_provide_map)
    ld.add_action(declare_show_rviz)
    ld.add_action(declare_use_emc)
    ld.add_action(declare_use_xtion)
    ld.add_action(teleop_launch)
    ld.add_action(xtion_launch)
    ld.add_action(service_tts)
    ld.add_action(emergency_manager)
    ld.add_action(rth_manager)
    ld.add_action(docking_manager)
    ld.add_action(battery_manager)
    ld.add_action(sound_manager)
    ld.add_action(map2odom)
    ld.add_action(odom2foot)
    ld.add_action(startup_sound_command)
    ld.add_action(rviz)

    # include and execute
    return ld
