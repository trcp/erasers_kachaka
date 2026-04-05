#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

import os



def generate_launch_description():
    ld = LaunchDescription()

    config_namespace = LaunchConfiguration("namespace")

    # declare arguments
    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="er_kachaka",
        description="Robot Namespace"
    )
    ld.add_action(declare_namespace)

    reliable_publisher = Node(
        package='erasers_kachaka_vision',
        executable='tof_camera_qos_conv',
        namespace=config_namespace
    )
    register_node = Node(
        package='depth_image_proc',
        executable='register_node',
        namespace=config_namespace,
        remappings=[
            ('rgb/camera_info', ['/', config_namespace, '/front_camera/reliable/camera_info']),
            ('depth/camera_info', ['/', config_namespace, '/tof_camera/reliable/camera_info']),
            ('depth/image_rect', ['/', config_namespace, '/tof_camera/reliable/image_raw']),
        ]
    )
    ld.add_action(reliable_publisher)
    ld.add_action(register_node)

    return ld
