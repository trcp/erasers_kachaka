#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction, GroupAction
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from ament_index_python.packages import get_package_share_directory

import os

KACHAKA_NAME = os.environ.get('KACHAKA_NAME')

BRINGUP_MSG = "Kachaka スタート！"


def generate_launch_description():
    ld = LaunchDescription()


    prefix_erk_teleop = get_package_share_directory("erasers_kachaka_teleop")
    prefix_erk_description = get_package_share_directory("erasers_kachaka_description")
    prefix_erk_manipulation = get_package_share_directory("erasers_kachaka_manipulation")
    prefix_rviz = os.path.join(
        get_package_share_directory("erasers_kachaka_bringup"),
        "rviz", "erasers_kachaka.rviz"
    )
    prefix_default_rviz = os.path.join(
        get_package_share_directory("erasers_kachaka_bringup"),
        "rviz", "erasers_kachaka_default.rviz"
    )


    # config
    config_bringup_type = LaunchConfiguration("bringup_type")
    config_use_rviz = LaunchConfiguration("use_rviz")
    config_shelf_type = LaunchConfiguration("shelf_type")


    # declare arguments
    declare_bringup_type = DeclareLaunchArgument(
        "bringup_type", default_value="1",
        description="起動する bridge コンテナの種類を選択します。[0, 1] のどちらかを選択してください。詳しくは起動方法ドキュメントを参照してください。"
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="True",
        description="Rviz2 を起動します"
    )
    declare_shelf_type = DeclareLaunchArgument(
        "shelf_type", default_value="1",
        description="[0, 1, 2] のどれかを選択してください。詳しくは起動方法ドキュメントを参照してください。"
    )

    ld.add_action(declare_bringup_type)
    ld.add_action(declare_shelf_type)
    ld.add_action(declare_use_rviz)


    # NODES
    node_kachaka_speak_subscriber = Node(
        package="erasers_kachaka_common",
        executable="kachaka_speak_subscriber",
        namespace=KACHAKA_NAME
    )
    node_emergency_manager = Node(
        package="erasers_kachaka_common",
        executable="emergency_manager",
        output="screen",
        namespace=KACHAKA_NAME
    )
    node_lidar_observer = Node(
        package="erasers_kachaka_common",
        executable="lidar_observer",
        output="screen",
        namespace=KACHAKA_NAME
    )
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", prefix_rviz],
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
    ld.add_action(node_lidar_observer)
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
            " up default_kachaka"
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
        period=5.0,
        actions=[
            bringup_trcp_docker,
            bringup_default_docker,
            bringup_msg
        ]
    )

    ld.add_action(bringup_actions)


    # LAUNCHERS
    launch_short_shelf_description = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            prefix_erk_description,
            "/launch/erasers_kachaka_description.launch"
        ]),
        condition=IfCondition(
            PythonExpression([
                config_shelf_type, " == 1",
                " or ",
                config_shelf_type, " == 2",
            ])
        )
    )
    launch_manipulation = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    prefix_erk_manipulation,
                    "/launch/manipulation_launch.py"
                ]),
                condition=IfCondition(
                    PythonExpression([
                        config_shelf_type, " == 2"
                    ])
                )
            )
        ]
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

    launchers = GroupAction(
        actions=[
            launch_short_shelf_description,
            launch_kachaka_description_only,
            launch_manipulation,
            launch_teleop
        ]
    )

    ld.add_action(launchers)


    return ld
