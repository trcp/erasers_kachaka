#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction, LogInfo, GroupAction
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from ament_index_python.packages import get_package_share_directory

import os


KACHAKA_NAME = os.environ.get('KACHAKA_NAME')
KACHAKA_IP = os.environ.get('KACHAKA_IP')
BRINGUP_TYPE = os.environ.get('BRINGUP_TYPE', 0)
SHELF_TYPE = os.environ.get('SHELF_TYPE', 2)
USE_SHELF = os.environ.get('USE_SHELF', False)
USE_RVIZ = os.environ.get('USE_RVIZ', False)
USE_TOF_POINTS = os.environ.get('USE_TOF_POINTS', True)
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
    prefix_default_rviz = os.path.join(
        get_package_share_directory("erasers_kachaka_bringup"),
        "rviz", "erasers_kachaka_default.rviz"
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
    config_namespace = LaunchConfiguration("namespace")
    config_ip = LaunchConfiguration("robot_ip")
    config_bringup_type = LaunchConfiguration("bringup_type")
    config_bringup_docker = LaunchConfiguration("bringup_docker")
    config_use_rviz = LaunchConfiguration("use_rviz")
    config_robot_description = LaunchConfiguration('robot_description')
    config_use_shelf = LaunchConfiguration('use_shelf')
    config_shelf_type = LaunchConfiguration("shelf_type")
    config_bringup_msg = LaunchConfiguration("bringup_msg")
    config_publish_tof_pc2 = LaunchConfiguration("publish_tof_pc2")


    # declare arguments
    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value=KACHAKA_NAME,
        description="Robot Namespace"
    )
    declare_ip = DeclareLaunchArgument(
        "robot_ip", default_value=KACHAKA_IP,
        description="Robot IP address"
    )
    declare_bringup_type = DeclareLaunchArgument(
        "bringup_type", default_value=BRINGUP_TYPE,
        description="Select bringup docker container type: [0, 1]. Please read doc about detail."
    )
    declare_bringup_docker = DeclareLaunchArgument(
        "bringup_docker", default_value="True",
        description="Launch docker container automatic."
    )
    declare_robot_descriptione = DeclareLaunchArgument(
        'robot_description', default_value=default_robot_description,
        description='Foll path for robot description'
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value=USE_RVIZ,
        description="Launch Rviz2"
    )
    declare_use_shelf = DeclareLaunchArgument(
        'use_shelf', default_value=USE_SHELF,
        description='Docking shelf'
    )
    declare_shelf_type = DeclareLaunchArgument(
        "shelf_type", default_value=SHELF_TYPE,
        description="Select shelf model type:[0, 1, 2]/ Please read doc about detail."
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
    ld.add_action(declare_bringup_docker)
    ld.add_action(declare_bringup_type)
    ld.add_action(declare_shelf_type)
    ld.add_action(declare_publish_tof_pc2)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_robot_descriptione)
    ld.add_action(declare_use_shelf)
    ld.add_action(declare_bringup_msg)


    # NODES
    node_kachaka_speak_subscriber = Node(
        package="erasers_kachaka_common",
        executable="kachaka_speak_subscriber",
        emulate_tty=True,
        namespace=config_namespace
    )
    node_emergency_manager = Node(
        package="erasers_kachaka_common",
        executable="emergency_manager",
        output="screen",
        emulate_tty=True,
        namespace=config_namespace
    )
    node_emergency_button = Node(
        package="erasers_kachaka_common",
        executable="emergency_button",
        output="screen",
        emulate_tty=True,
        namespace=config_namespace
    )
    node_battery_manager = Node(
        package="erasers_kachaka_common",
        executable="battery_manager",
        output="screen",
        emulate_tty=True,
        namespace=config_namespace
    )
    node_volume_manager = Node(
        package="erasers_kachaka_common",
        executable="volume_manager",
        output="screen",
        emulate_tty=True,
        parameters=[{'kachaka_ip': config_ip}],
        namespace=config_namespace
    )
    node_dock_manager = Node(
        package="erasers_kachaka_common",
        executable="dock_manager",
        output="screen",
        emulate_tty=True,
        parameters=[{'kachaka_ip': config_ip}],
        namespace=config_namespace
    )
    node_object_detection_visualizer = Node(
        package="erasers_kachaka_vision",
        executable="object_detection_visualizer",
        output="screen",
        emulate_tty=True,
        namespace=config_namespace
    )
    node_lidar_observer = Node(
        package="erasers_kachaka_common",
        executable="lidar_observer",
        output="screen",
        emulate_tty=True,
        namespace=config_namespace
    )
    node_robot_stopper = Node(
        package="erasers_kachaka_common",
        executable="robot_stopper",
        output="screen",
        emulate_tty=True,
        namespace=config_namespace
    )
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", prefix_rviz],
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression([
                config_bringup_type, " == 0 ",
                " and ",
                config_use_rviz
            ])
        )
    )
    node_default_rviz = Node(
        package="rviz2",
        executable="rviz2",
        emulate_tty=True,
        arguments=["-d", prefix_default_rviz],
        condition=IfCondition(
            PythonExpression([
                config_bringup_type, " == 1",
                " and ",
                config_use_rviz
            ])
        )
    )
    node_mapprovider = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="own_log",
        condition=IfCondition(
            PythonExpression([
                config_bringup_type, " == 0"
            ])
        )
    )


    # PROCESS
    bringup_trcp_docker = ExecuteProcess(
        cmd=[[
            "docker compose",
            " -f %s/compose.yaml"%os.environ.get('KACHAKA_ERK_PATH'),
            " up nomap_bridge"
        ]],
        shell=True,
        condition=IfCondition(
            PythonExpression([
                config_bringup_type, " == 0",
                " and ",
                config_bringup_docker
            ])
        )
    )
    bringup_default_docker = ExecuteProcess(
        cmd=[[
            "docker compose",
            " -f %s/compose.yaml"%os.environ.get('KACHAKA_ERK_PATH'),
            " up official_bridge"
        ]],
        shell=True,
        condition=IfCondition(
            PythonExpression([
                config_bringup_type, " == 1",
                " and ",
                config_bringup_docker
            ])
        )
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
                        " \"{data: \'%s\'}\""%BRINGUP_MSG
                    ]],
                    shell=True
                )
            ]
        )
    )

    bringup_actions = TimerAction(
        period=1.5,
        actions=[
            bringup_trcp_docker,
            bringup_default_docker,
        ]
    )

    loggers = GroupAction(
        actions=[
            LogInfo(msg="============== eR@sers Kachaka Info =================="),
            LogInfo(msg=["Kachaka Name: ", config_namespace]),
            LogInfo(msg=["Kachaka IP: ", config_ip]),
            LogInfo(msg=["Bringup Type: ", config_bringup_type]),
            LogInfo(msg=["Use Shelf: ", config_use_shelf]),
            LogInfo(msg=["Shelf Type: ", config_shelf_type]),
            LogInfo(msg="======================================================"),
        ]
    )

    ld.add_action(bringup_actions)
    ld.add_action(loggers)


    # LAUNCHERS
    launch_kachaka_description =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("erasers_kachaka_description"),
            "/launch/description.launch.py"
        ]),
        launch_arguments={
            "namespace":config_namespace,
            "robot_description":config_robot_description,
            "use_shelf":config_use_shelf,
            "shelf_type":config_shelf_type,
            "use_shelf":"false",
        }.items(),
    )
                   

    launch_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            prefix_erk_teleop,
            "/launch/teleop.launch.py"
        ])
    )

    launch_tof_pointcloud = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            prefix_erk_vision,
            "/launch/tof_pointcloud_launch.py"
        ]),
        launch_arguments={
            "namespace":config_namespace,
        }.items(),
        condition=IfCondition(config_publish_tof_pc2)
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
            node_lidar_observer,
            node_robot_stopper,
            node_rviz,
            node_default_rviz,
            node_mapprovider,
            # launchers
            launch_kachaka_description,
            launch_teleop,
            launch_tof_pointcloud
        ]
    )
    ld.add_action(erasers_kachaka_bringup)


    return ld
