#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from ament_index_python.packages import get_package_share_directory

import os

#KACHAKA_NAME = "container_kachaka"
KACHAKA_NAME = os.environ.get('KACHAKA_NAME')
BRINGUP_MSG = os.environ.get('BRINGUP_MSG')

if BRINGUP_MSG == None:
    BRINGUP_MSG = "Kachaka!スタート!"


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


    # config
    config_bringup_type = LaunchConfiguration("bringup_type")
    config_use_rviz = LaunchConfiguration("use_rviz")
    config_shelf_type = LaunchConfiguration("shelf_type")
    config_bringup_msg = LaunchConfiguration("bringup_msg")
    config_publish_tof_pc2 = LaunchConfiguration("publish_tof_pc2")
    config_robot_name = LaunchConfiguration("robot_name")

    # declare arguments
    declare_bringup_type = DeclareLaunchArgument(
        "bringup_type", default_value="1",
        description="起動する bridge コンテナの種類を選択します。[0, 1] のどちらかを選択してください。詳しくは起動方法ドキュメントを参照してください。"
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="False",
        description="Rviz2 を起動します True or False"
    )
    declare_shelf_type = DeclareLaunchArgument(
        "shelf_type", default_value="1",
        description="[0, 1, 2] のどれかを選択してください。詳しくは起動方法ドキュメントを参照してください。"
    )
    declare_bringup_msg = DeclareLaunchArgument(
        "bringup_msg", default_value=BRINGUP_MSG,
        description="カチャカ起動時のメッセージを設定します。"
    )
    declare_publish_tof_pc2 = DeclareLaunchArgument(
        "publish_tof_pc2", default_value="True",
        description="カチャカ前方 ToF センサーの PointCloud2 を発行します。"
    )


    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="container_kachaka", # デフォルト値を設定
        description="add container ver."
    )

    ld.add_action(declare_bringup_type)
    ld.add_action(declare_shelf_type)
    ld.add_action(declare_publish_tof_pc2)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_robot_name)

    # NODES
    node_kachaka_speak_subscriber = Node(
        package="erasers_kachaka_common",
        executable="kachaka_speak_subscriber",
        emulate_tty=True,
        namespace=config_robot_name
    )
    node_emergency_manager = Node(
        package="erasers_kachaka_common",
        executable="emergency_manager",
        output="screen",
        emulate_tty=True,
        namespace=config_robot_name
    )
    node_emergency_button = Node(
        package="erasers_kachaka_common",
        executable="emergency_button",
        output="screen",
        emulate_tty=True,
        namespace=config_robot_name
    )
    node_battery_manager = Node(
        package="erasers_kachaka_common",
        executable="battery_manager",
        output="screen",
        emulate_tty=True,
        namespace=config_robot_name
    )
    node_lidar_observer = Node(
        package="erasers_kachaka_common",
        executable="lidar_observer",
        output="screen",
        emulate_tty=True,
        namespace=config_robot_name
    )
    node_lidar_resampler = Node(
        package="erasers_kachaka_common",
        executable="lidar_resampler",
        output="screen",
        emulate_tty=True,
        namespace=config_robot_name,
        remappings=[
            ("input_scan", "lidar/scan"),
            ("output_scan", "sampling_lidar/scan")
        ]
    )
    node_pt_field = Node(
        package="erasers_kachaka_navigation",
        executable="pot_fields_node",
        output="screen",
        parameters=[param_for_pt_fields_node],
        emulate_tty=True,
        namespace=config_robot_name
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

    ld.add_action(node_kachaka_speak_subscriber)
    ld.add_action(node_emergency_manager)
    ld.add_action(node_emergency_button)
    ld.add_action(node_battery_manager)
    ld.add_action(node_lidar_observer)
    ld.add_action(node_lidar_resampler)
    ld.add_action(node_pt_field)
    ld.add_action(node_rviz)
    ld.add_action(node_default_rviz)
    ld.add_action(node_mapprovider)


    # PROCESS
    bringup_trcp_docker = ExecuteProcess(
        cmd=[[
            "docker compose",
            " -f %s/docker/docker-compose.yaml"%os.environ.get('KACHAKA_ERK_PATH'),
            " up kachaka"
        ]],
        shell=True,
        condition=IfCondition(
            PythonExpression([
                config_bringup_type, " == 0"
            ])
        )
    )
    bringup_default_docker = ExecuteProcess(
        cmd=[[
            "docker compose",
            " -f %s/docker/docker-compose.yaml"%os.environ.get('KACHAKA_ERK_PATH'),
            " up container_kachaka"
        ]],
        shell=True,
        condition=IfCondition(
            PythonExpression([
                config_bringup_type, " == 1"
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
                        " \"{data: %s}\""%BRINGUP_MSG
                    ]],
                    shell=True
                )
            ]
        )
    )

    bringup_actions = TimerAction(
        period=2.0,
        actions=[
            bringup_trcp_docker,
            bringup_default_docker,
        ]
    )

    ld.add_action(bringup_actions)
    ld.add_action(bringup_msg)


    # LAUNCHERS
    launch_short_shelf_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            prefix_erk_description,
            "/launch/erasers_kachaka_container_description.launch.py"
        ]),
        launch_arguments={
            "namespace":"",
            "frame_prefix": config_robot_name,
        }.items(),
        condition=IfCondition(
            PythonExpression([
                config_shelf_type, " == 1",
                #" or ",
                #config_shelf_type, " == 2",
            ])
        )
    )
    launch_kachaka_description_only =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("kachaka_description"),
            "/launch/robot_description.launch.py"
        ]),
        launch_arguments={
            "namespace":"",
            "frame_prefix":"",
        }.items(),
        condition=IfCondition(
            PythonExpression([
                config_shelf_type, " == 0"
            ])
        )
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
        condition=IfCondition(config_publish_tof_pc2)
    )


    ld.add_action(launch_short_shelf_description)
    ld.add_action(launch_kachaka_description_only)
    ld.add_action(launch_teleop)
    ld.add_action(launch_tof_pointcloud)


    return ld
