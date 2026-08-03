#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
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
            ('depth_registered/camera_info', ['/', config_namespace, '/tof_camera/registered/camera_info']),
            ('depth_registered/image_rect', ['/', config_namespace, '/tof_camera/registered/image_rect']),
        ]
    )
    xyzrgb_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',
        name='point_cloud_xyzrgb_node',
        namespace=config_namespace,
        remappings=[
            ('rgb/camera_info', ['/', config_namespace, '/front_camera/reliable/camera_info']),
            ('rgb/image_rect_color', ['/', config_namespace, '/front_camera/reliable/image_raw']),
            ('depth_registered/image_rect', ['/', config_namespace, '/tof_camera/registered/image_rect']),
            ('points', ['/', config_namespace, '/tof_camera/registered/points']),
        ],
    )

    node_container = ComposableNodeContainer(
        name="container",
        namespace=config_namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package="depth_image_proc",
                plugin='depth_image_proc::PointCloudXyzNode',
                name='point_cloud_xyz_node',
                namespace=config_namespace,
                remappings=[
                    ("camera_info", "tof_camera/reliable/camera_info"),
                    ("image_rect", "tof_camera/reliable/image_raw"),
                    ("points", "tof_camera/points")
                ]
            )
        ]
    )

    ld.add_action(reliable_publisher)
    ld.add_action(register_node)
    ld.add_action(xyzrgb_node)
    #ld.add_action(node_container)


    return ld
