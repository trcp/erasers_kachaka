#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os


def generate_launch_description():
    ld = LaunchDescription()

    # configures
    server_uri = LaunchConfiguration('server_uri')
    namespace = LaunchConfiguration('namespace')
    frame_prefix = LaunchConfiguration('frame_prefix')


    # launch arguments
    declare_server_uri = DeclareLaunchArgument(
        'server_uri', default_value=os.environ['API_GRPC_BRIDGE_SERVER_URI']
    )
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value=os.environ['KACHAKA_NAME']
    )
    declare_frame_prefix = DeclareLaunchArgument(
        'frame_prefix', default_value=os.environ['KACHAKA_NAME']+'_'
    )
    ld.add_action(declare_server_uri)
    ld.add_action(declare_namespace)
    ld.add_action(declare_frame_prefix)


    common_parameters = {'server_uri': server_uri}
    frame_parameters = {
        'server_uri': server_uri,
        'frame_prefix': frame_prefix,
    }


    # Nodes
    node_auto_homing = ComposableNode(
        package='kachaka_grpc_ros2_bridge',
        plugin='kachaka::grpc_ros2_bridge::AutoHomingComponent',
        name='auto_homing',
        namespace=namespace,
        parameters=[common_parameters],
        extra_arguments=[{'use_intra_process_comms': False}],
    )
    node_back_camera = ComposableNode(
        package='kachaka_grpc_ros2_bridge',
        plugin='kachaka::grpc_ros2_bridge::BackCameraComponent',
        name='back_camera',
        namespace=namespace,
        parameters=[frame_parameters],
        extra_arguments=[{'use_intra_process_comms': False}],
    )
    node_kachaka_command = ComposableNode(
        package='kachaka_grpc_ros2_bridge',
        plugin='kachaka::grpc_ros2_bridge::KachakaCommandComponent',
        name='kachaka_command',
        namespace=namespace,
        parameters=[common_parameters],
        extra_arguments=[{'use_intra_process_comms': False}],
    )
    node_front_camera = ComposableNode(
        package='kachaka_grpc_ros2_bridge',
        plugin='kachaka::grpc_ros2_bridge::FrontCameraComponent',
        name='front_camera',
        namespace=namespace,
        parameters=[frame_parameters],
        extra_arguments=[{'use_intra_process_comms': False}],
    )
    node_imu = ComposableNode(
        package='kachaka_grpc_ros2_bridge',
        plugin='kachaka::grpc_ros2_bridge::ImuComponent',
        name='imu',
        namespace=namespace,
        parameters=[frame_parameters],
        extra_arguments=[{'use_intra_process_comms': False}],
    )
    node_lidar = ComposableNode(
        package='kachaka_grpc_ros2_bridge',
        plugin='kachaka::grpc_ros2_bridge::LidarComponent',
        name='lidar',
        namespace=namespace,
        parameters=[frame_parameters],
        remappings=[
            ('~/scan', 'lidar/scan_raw'),
        ],
        extra_arguments=[{'use_intra_process_comms': False}],
    )
    node_manual_control = ComposableNode(
        package='kachaka_grpc_ros2_bridge',
        plugin='kachaka::grpc_ros2_bridge::ManualControlComponent',
        name='manual_control',
        namespace=namespace,
        parameters=[common_parameters],
        extra_arguments=[{'use_intra_process_comms': False}],
    )
    node_robot_info = ComposableNode(
        package='kachaka_grpc_ros2_bridge',
        plugin='kachaka::grpc_ros2_bridge::RobotInfoComponent',
        name='robot_info',
        namespace=namespace,
        parameters=[common_parameters],
        extra_arguments=[{'use_intra_process_comms': False}],
    )
    node_object_detection = ComposableNode(
        package='kachaka_grpc_ros2_bridge',
        plugin='kachaka::grpc_ros2_bridge::ObjectDetectionComponent',
        name='object_detection',
        namespace=namespace,
        parameters=[common_parameters],
        extra_arguments=[{'use_intra_process_comms': False}],
    )
    node_wheel_odometry = ComposableNode(
        package='kachaka_grpc_ros2_bridge',
        plugin='kachaka::grpc_ros2_bridge::WheelOdometryComponent',
        name='wheel_odometry',
        namespace=namespace,
        parameters=[frame_parameters],
        remappings=[
            ('~/wheel_odometry', 'odometry/odometry_raw'),
        ],
        extra_arguments=[{'use_intra_process_comms': False}],
    )
    node_tof_camera = ComposableNode(
        package='kachaka_grpc_ros2_bridge',
        plugin='kachaka::grpc_ros2_bridge::TofCameraComponent',
        name='tof_camera',
        namespace=namespace,
        parameters=[frame_parameters],
        extra_arguments=[{'use_intra_process_comms': False}],
    )
    node_torch = ComposableNode(
        package='kachaka_grpc_ros2_bridge',
        plugin='kachaka::grpc_ros2_bridge::TorchComponent',
        name='torch',
        namespace=namespace,
        parameters=[common_parameters],
        extra_arguments=[{'use_intra_process_comms': False}],
    )
    node_dynamic_tf = ComposableNode(
        package='kachaka_grpc_ros2_bridge',
        plugin='kachaka::grpc_ros2_bridge::DynamicTfComponent',
        name='dynamic_tf',
        namespace=namespace,
        parameters=[frame_parameters],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    node_grpc_ros2_bridge_container = ComposableNodeContainer(
        name='grpc_ros2_bridge_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container_mt',
        emulate_tty=True,
        composable_node_descriptions=[
            node_auto_homing,
            node_back_camera,
            node_kachaka_command,
            node_front_camera,
            node_imu,
            node_lidar,
            node_manual_control,
            node_robot_info,
            node_object_detection,
            node_wheel_odometry,
            node_tof_camera,
            node_torch,
            node_dynamic_tf,
        ],
    )
    ld.add_action(node_grpc_ros2_bridge_container)


    return ld
