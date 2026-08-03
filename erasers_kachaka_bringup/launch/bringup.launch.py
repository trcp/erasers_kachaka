#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction, LogInfo, GroupAction
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os


KACHAKA_NAME = os.environ.get('KACHAKA_NAME')
KACHAKA_IP = os.environ.get('KACHAKA_IP')
SHELF_TYPE = os.environ.get('SHELF_TYPE', '0')
USE_RVIZ = os.environ.get('USE_RVIZ', 'False')
USE_TOF_POINTS = os.environ.get('USE_TOF_POINTS', 'True')
USE_EMC = os.environ.get('USE_EMC', 'False')
BRINGUP_MSG = os.environ.get('BRINGUP_MSG', 'erasers_kachaka, start! DOMAIN number is %s'%os.environ.get('ROS_DOMAIN_ID', 0))


def generate_launch_description():
    ld = LaunchDescription()


    prefix_erk_teleop = get_package_share_directory("erasers_kachaka_teleop")
    prefix_erk_vision = get_package_share_directory("erasers_kachaka_vision")
    prefix_erk_description = get_package_share_directory("erasers_kachaka_description")
    prefix_erk_navigation = get_package_share_directory("erasers_kachaka_navigation")

    prefix_rviz = os.path.join(
        get_package_share_directory("erasers_kachaka_bringup"),
        "rviz", "erasers_kachaka.rviz"
    )
    ekf_params_file = os.path.join(
        get_package_share_directory("erasers_kachaka_bringup"),
        "params", "ekf.yaml"
    )
    param_for_pt_fields_node = os.path.join(
        prefix_erk_navigation, 'params',
        'pt_fields.yaml'
    )
    param_for_leg_finder_node = os.path.join(
        prefix_erk_navigation, 'params',
        'leg_finder.yaml'
    )
    default_robot_description = os.path.join(
        get_package_share_directory('erasers_kachaka_description'),
        'urdf', 'kachaka.urdf.xacro'
    )


    # config
    namespace = LaunchConfiguration("namespace")
    ip = LaunchConfiguration("robot_ip")
    frame_prefix = LaunchConfiguration("frame_prefix")
    robot_description = LaunchConfiguration('robot_description')
    use_rviz = LaunchConfiguration("use_rviz")
    use_emc = LaunchConfiguration("use_emc")
    shelf_type = LaunchConfiguration("shelf_type")
    bringup_msg = LaunchConfiguration("bringup_msg")
    publish_tof_pc2 = LaunchConfiguration("publish_tof_pc2")


    # declare arguments
    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value=KACHAKA_NAME,
        description="Robot Namespace"
    )
    declare_ip = DeclareLaunchArgument(
        "robot_ip", default_value=KACHAKA_IP,
        description="Robot IP address"
    )
    declare_frame_prefix = DeclareLaunchArgument(
        "frame_prefix", default_value=KACHAKA_NAME + "_",
        description="TF frame prefix"
    )
    declare_robot_descriptione = DeclareLaunchArgument(
        'robot_description', default_value=default_robot_description,
        description='Foll path for robot description'
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value=USE_RVIZ,
        description="Launch Rviz2"
    )
    declare_use_emc = DeclareLaunchArgument(
        'use_emc', default_value=USE_EMC,
        description='Enable Emergency Button'
    )
    declare_shelf_type = DeclareLaunchArgument(
        "shelf_type", default_value=SHELF_TYPE,
        description="Select shelf model type: [0, 2, 3]."
    )
    declare_bringup_msg = DeclareLaunchArgument(
        "bringup_msg", default_value=BRINGUP_MSG,
        description="Define speak bringup message from Kachaka."
    )
    declare_publish_tof_pc2 = DeclareLaunchArgument(
        "publish_tof_pc2", default_value=USE_TOF_POINTS,
        description="Enable publish TOF Pointcloud2 topic from Kachaka front sensor."
    )

    ld.add_action(declare_namespace)
    ld.add_action(declare_ip)
    ld.add_action(declare_frame_prefix)
    ld.add_action(declare_shelf_type)
    ld.add_action(declare_publish_tof_pc2)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_robot_descriptione)
    ld.add_action(declare_use_emc)
    ld.add_action(declare_bringup_msg)


    # NODES
    node_kachaka_speak_subscriber = Node(
        package="erasers_kachaka_common",
        executable="kachaka_speak_subscriber",
        emulate_tty=True,
        namespace=namespace
    )
    node_emergency_manager = Node(
        package="erasers_kachaka_common",
        executable="emergency_manager",
        output="screen",
        emulate_tty=True,
        namespace=namespace,
        condition=IfCondition(use_emc)
    )
    node_emergency_button = Node(
        package="erasers_kachaka_common",
        executable="emergency_button",
        output="screen",
        emulate_tty=True,
        namespace=namespace,
        condition=IfCondition(use_emc)
    )
    node_battery_manager = Node(
        package="erasers_kachaka_common",
        executable="battery_manager",
        output="screen",
        emulate_tty=True,
        namespace=namespace
    )
    node_volume_manager = Node(
        package="erasers_kachaka_common",
        executable="volume_manager",
        output="screen",
        emulate_tty=True,
        parameters=[{'kachaka_ip': ip}],
        namespace=namespace
    )
    node_dock_manager = Node(
        package="erasers_kachaka_common",
        executable="dock_manager",
        output="screen",
        emulate_tty=True,
        parameters=[{'kachaka_ip': ip}],
        namespace=namespace
    )
    node_object_detection_visualizer = Node(
        package="erasers_kachaka_vision",
        executable="object_detection_visualizer",
        output="screen",
        emulate_tty=True,
        namespace=namespace
    )
    node_lidar_resampler = Node(
        package="erasers_kachaka_common",
        executable="lidar_resampler",
        name="lidar_resampler",
        namespace=namespace,
        output="screen",
        emulate_tty=True,
        parameters=[{
            "target_points": 0,
            "input_scan_topic": "lidar/scan_raw",
            "output_scan_topic": "lidar/scan",
        }],
    )
    node_lidar_observer = Node(
        package="erasers_kachaka_common",
        executable="lidar_observer",
        output="screen",
        emulate_tty=True,
        namespace=namespace
    )
    node_robot_stopper = Node(
        package="erasers_kachaka_common",
        executable="robot_stopper",
        output="screen",
        emulate_tty=True,
        namespace=namespace
    )
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d", prefix_rviz,
            "-f", [frame_prefix, "odom"]
        ],
        emulate_tty=True,
        condition=IfCondition(use_rviz)
    )
    node_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        namespace=namespace,
        output="screen",
        emulate_tty=True,
        parameters=[
            ekf_params_file,
            {
                "odom_frame": [frame_prefix, "odom"],
                "world_frame": [frame_prefix, "odom"],
                "base_link_frame": [frame_prefix, "base_footprint"],
            },
        ],
        remappings=[
            ("odometry/filtered", "odometry/odometry"),
        ],
    )


    # PROCESS
    bringup_msg = RegisterEventHandler(
        OnProcessStart(
            target_action=node_kachaka_speak_subscriber,
            on_start=[
                ExecuteProcess(
                    cmd=[[
                        "ros2 topic pub --once",
                        " /%s/kachaka_speak"%KACHAKA_NAME,
                        " std_msgs/msg/String",
                        " \"{data: \'%s\'}\""%BRINGUP_MSG
                    ]],
                    shell=True
                )
            ]
        )
    )

    loggers = GroupAction(
        actions=[
            LogInfo(msg="============== eR@sers Kachaka Info =================="),
            LogInfo(msg=["Kachaka Name: ", namespace]),
            LogInfo(msg=["Kachaka IP: ", ip]),
            LogInfo(msg=["Shelf Type: ", shelf_type]),
            LogInfo(msg="======================================================"),
        ]
    )

    ld.add_action(loggers)


    # LAUNCHERS
    launch_kachaka_description =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("erasers_kachaka_description"),
            "/launch/description.launch.py"
        ]),
        launch_arguments={
            "namespace":namespace,
            "frame_prefix":frame_prefix,
            "robot_description":robot_description,
            "shelf_type":shelf_type,
        }.items(),
    )

    launch_grpc_ros2_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("erasers_kachaka_bringup"),
            "/launch/grpc_ros2_bridge.trcp.launch.py"
        ]),
        launch_arguments={
            "namespace":namespace,
            "frame_prefix":frame_prefix,
        }.items(),
    )
                   

    launch_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            prefix_erk_teleop,
            "/launch/teleop.launch.py"
        ]),
        launch_arguments={
            "namespace":namespace,
            "use_emc":use_emc
        }.items(),
    )

    launch_tof_pointcloud = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            prefix_erk_vision,
            "/launch/tof_pointcloud.launch.py"
        ]),
        launch_arguments={
            "namespace":namespace,
        }.items(),
        condition=IfCondition(publish_tof_pc2)
    )


    erasers_kachaka_bringup = TimerAction(
        period=5.0,
        actions=[
            bringup_msg,
            # nodes
            node_kachaka_speak_subscriber,
            node_emergency_manager,
            node_emergency_button,
            node_battery_manager,
            node_volume_manager,
            node_dock_manager,
            node_object_detection_visualizer,
            node_lidar_resampler,
            node_lidar_observer,
            node_robot_stopper,
            node_rviz,
            node_ekf,
            # launchers
            launch_grpc_ros2_bridge,
            launch_kachaka_description,
            #launch_teleop,
            launch_tof_pointcloud
        ]
    )
    bringup_msg_bringup = TimerAction(
        period=7.0,
        actions=[bringup_msg]
    )
    ld.add_action(erasers_kachaka_bringup)
    #ld.add_action(bringup_msg_bringup)


    return ld
