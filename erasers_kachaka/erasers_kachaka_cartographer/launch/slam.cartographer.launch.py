#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    ld = LaunchDescription()

    ### prefix
    erasers_kachaka_cartographer = get_package_share_directory("erasers_kachaka_cartographer")
    param_dir = os.path.join(erasers_kachaka_cartographer, "params")
    param_file = "cartographer.lua"

    ### configurations
    namespace = LaunchConfiguration("namespace", default="er_kachaka/localization")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    configuration_directory = LaunchConfiguration("configuration_directory", default=param_dir)
    configuration_basename = LaunchConfiguration("configuration_basename", default=param_file)
    resolution = LaunchConfiguration("resolution", default="0.025")
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    use_navigation = LaunchConfiguration("use_navigation")
    auto_map_save = LaunchConfiguration("auto_map_save")

    ### declare argument
    declare_configuration_directory = DeclareLaunchArgument(
        "configuration_directory", default_value=param_dir,
        description="cartographer パラメータファイルが存在するディレクトリを指定します。"
    )
    declare_configuration_basename = DeclareLaunchArgument(
        "configuration_basename", default_value=param_file,
        description="cartographer パラメータファイル名を指定します。"
    )
    declare_resolution = DeclareLaunchArgument(
        "resolution", default_value=resolution,
        description="生成されるマップの解像度を指定します。"
    )
    declare_publish_period_sec = DeclareLaunchArgument(
        "publish_period_sec", default_value=publish_period_sec,
        description="マップの更新頻度を指定します。"
    )
    declare_use_navigation = DeclareLaunchArgument(
        "use_navigation", default_value="true",
        description="ナビゲーションを有効にします。"
    )
    declare_auto_map_save = DeclareLaunchArgument(
        "auto_map_save", default_value="true",
        description="自動的にマップを保存します。"
    )

    ld.add_action(declare_configuration_directory)
    ld.add_action(declare_configuration_basename)
    ld.add_action(declare_resolution)
    ld.add_action(declare_publish_period_sec)
    ld.add_action(declare_use_navigation)
    ld.add_action(declare_auto_map_save)

    ### execute navigation
    launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("erasers_kachaka_navigation"),
                "/launch/navigation.nav2.launch.py"
            ]
        ),
        launch_arguments={
            "namespace":"/er_kachaka/localization"
        }.items(),
        condition=IfCondition(use_navigation)
    )

    ld.add_action(launch_navigation)

    ### Node
    cartographer = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name='cartographer_node',
        namespace=namespace,
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename],
        remappings=[('scan', '/er_kachaka/lidar/scan'),
                    ('odom', '/er_kachaka/odometry/odometry'),
                    ('imu', '/er_kachaka/imu/imu')],
    )
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
    )

    auto_map_saver = Node(
        package="erasers_kachaka_cartographer",
        executable="auto_map_saver",
        namespace=namespace,
        condition=IfCondition(auto_map_save)
    )

    ld.add_action(cartographer)
    ld.add_action(occupancy_grid_node)
    ld.add_action(auto_map_saver)

    return ld
