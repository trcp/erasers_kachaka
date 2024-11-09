#!/usr/bin/env python3
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # prefix
    navigation_prefix = get_package_share_directory("erasers_kachaka_navigation")
    cartographer_prefix = get_package_share_directory("erasers_kachaka_cartographer")
    map_dir_prefix = os.path.join(cartographer_prefix, "map")

    use_localization = LaunchConfiguration("use_localization")
    map_name = LaunchConfiguration("map_name")
    log = LaunchConfiguration("log")

    map_yaml = PythonExpression(["'", LaunchConfiguration("map_name"), "' + '.yaml'"])
    map = PathJoinSubstitution([cartographer_prefix, "map", map_yaml])

    logger = LogInfo(msg=map)
    declare_use_localization = DeclareLaunchArgument(
        "use_localization",
        default_value="false",
        description="ローカリゼーションを他で使用する場合は True にしてください"
    )
    declare_map_name = DeclareLaunchArgument(
        "map_name",
        default_value="map",
        description="使用したいマップ名を指定してくだい。"
    )
    declare_log = DeclareLaunchArgument(
        "log",
        default_value="screen",
        description="ログ出力を指定します。"
    )

    launch_emcl2_localization = TimerAction(
        period=10.0,
        condition=UnlessCondition(use_localization),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        navigation_prefix,
                        "/launch/localization.emcl.launch.py"
                    ]
                ),
                launch_arguments={
                    "map": map,
                    "log":log
                }.items()
            )
        ]
    )

    launch_navigation2_via_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                navigation_prefix,
                "/launch/navigation.nav2.launch.py"
            ]
        ),
        launch_arguments={
            "namespace":"/er_kachaka/navigation",
            "log":log
        }.items(),
        condition=UnlessCondition(use_localization)
    )

    launch_navigation2_via_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                navigation_prefix,
                "/launch/navigation.nav2.launch.py"
            ]
        ),
        launch_arguments={
            "namespace":"/er_kachaka/localization",
            "log":log
        }.items(),
        condition=IfCondition(use_localization)
    )

    ld.add_action(declare_use_localization)
    ld.add_action(declare_map_name)
    ld.add_action(declare_log)
    ld.add_action(logger)
    ld.add_action(launch_emcl2_localization)
    ld.add_action(launch_navigation2_via_localization)
    ld.add_action(launch_navigation2_via_navigation)

    return ld
    
