#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()


    default_controller_file = os.path.join(get_package_share_directory('erasers_kachaka_teleop'), 'params', 'nintendo_pro_controller.yaml')


    namespace = LaunchConfiguration("namespace")
    use_emc = LaunchConfiguration('use_emc')
    controller_file = LaunchConfiguration('controller_file')


    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value='er_kachaka',
        description="Robot Namespace"
    )
    declare_use_emc = DeclareLaunchArgument(
        'use_emc', default_value='false',
        description='enable emergency button'
    )
    declare_controller_file = DeclareLaunchArgument(
        'controller_file', default_value=default_controller_file,
        description='Map of controller button'
    )
    ld.add_action(declare_namespace)
    ld.add_action(declare_use_emc)
    ld.add_action(declare_controller_file)


    node_emergency_button = Node(
        package="joy",
        executable="joy_node",
        emulate_tty=True,
        parameters=[
            {'device_id': 0}
        ],
        namespace=[namespace, '/emergency'],
        condition=IfCondition(use_emc)
    )
    node_joy_with_emc = Node(
        package="joy",
        executable="joy_node",
        emulate_tty=True,
        parameters=[
            {'device_id': 1}
        ],
        namespace=namespace,
        condition=IfCondition(use_emc)
    )
    node_joy_without_emc = Node(
        package="joy",
        executable="joy_node",
        parameters=[
            {'device_id': 0}
        ],
        namespace=namespace,
        condition=UnlessCondition(use_emc)
    )
    node_teleop = Node(
        package="erasers_kachaka_teleop",
        executable="teleop",
        parameters=[controller_file],
        namespace=namespace
    )

    ld.add_action(node_emergency_button)
    ld.add_action(node_joy_with_emc)
    ld.add_action(node_joy_without_emc)
    ld.add_action(node_teleop)
    
    return ld
