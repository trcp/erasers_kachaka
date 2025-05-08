#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import os


KACHAKA_NAME = os.environ.get('KACHAKA_NAME')


def generate_launch_description():
    ld = LaunchDescription()

    node_container = ComposableNodeContainer(
        name="container",
        namespace=KACHAKA_NAME,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package="depth_image_proc",
                plugin='depth_image_proc::PointCloudXyzNode',
                name='point_cloud_xyz_node',
                namespace=KACHAKA_NAME,
                parameters=[{'queue_size': 10}],
                remappings=[
                    ("camera_info", "tof_camera/camera_info"),
                    ("image_rect", "tof_camera/image_raw"),
                    ("points", "tof_camera/points")
                ]
            )
        ]
    )

    ld.add_action(node_container)


    return ld
