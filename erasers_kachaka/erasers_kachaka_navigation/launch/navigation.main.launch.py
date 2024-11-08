#!/usr/bin/env python3
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # prefix
    navigation_prefix = get_package_share_directory("erasers_kachaka_navigation")

    use_localization = LaunchConfiguration("use_localization")

    declare_use_localization = DeclareLaunchArgument(
        "use_localization",
        default_value="false",
        description="ローカリゼーションを他で使用する場合は True にしてください"
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
                )
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
            "namespace":"/er_kachaka/navigation"
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
            "namespace":"/er_kachaka/localization"
        }.items(),
        condition=IfCondition(use_localization)
    )

    ld.add_action(declare_use_localization)
    ld.add_action(launch_emcl2_localization)
    ld.add_action(launch_navigation2_via_localization)
    ld.add_action(launch_navigation2_via_navigation)

    return ld
    