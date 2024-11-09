#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    front_namespace = "er_kachaka/front_camera"
    front_image_topic = "/er_kachaka/front_camera/image_raw"
    front_camerainfo_topic = "/er_kachaka/front_camera/camera_info"
    back_namespace = "er_kachaka/back_camera"
    back_image_topic = "/er_kachaka/back_camera/image_raw"
    back_camerainfo_topic = "/er_kachaka/back_camera/camera_info"
    marker_size = 0.04
    aruco_dictionary_id = "DICT_4X4_100"

    aruco_marker_detect_via_front =Node(
        package='ros2_aruco',  
        executable='aruco_node',      
        name='front_aruco_node',
        namespace=front_namespace,
        parameters=[{
            'marker_size': marker_size,
            'aruco_dictionary_id': aruco_dictionary_id,
            'image_topic': front_image_topic,
            'camera_info_topic': front_camerainfo_topic
        }],
        output='screen'
    )

    aruco_marker_detect_via_back =Node(
        package='ros2_aruco',  
        executable='aruco_node',      
        name='back_aruco_node',
        namespace=back_namespace,
        parameters=[{
            'marker_size': marker_size,
            'aruco_dictionary_id': aruco_dictionary_id,
            'image_topic': back_image_topic,
            'camera_info_topic': back_camerainfo_topic
        }],
        output='screen'
    )

    ld.add_action(aruco_marker_detect_via_front)
    ld.add_action(aruco_marker_detect_via_back)

    return ld
