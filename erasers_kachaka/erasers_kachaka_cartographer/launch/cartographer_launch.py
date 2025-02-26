#!/usr/bin/env python3
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


NAMESPACE = os.environ.get('KACHAKA_NAME')


def generate_launch_description():

    ld = LaunchDescription()

    # 各コンフィグのデフォルト値
    default_map_dir = "/home/roboworks/colcon_ws/src/erasers_kachaka/erasers_kachaka/erasers_kachaka_cartographer/map"
    default_map_name = "test_field"
    defaulr_map_save_late = "5"
    default_config_dir = os.path.join(
        get_package_share_directory("erasers_kachaka_cartographer"),"config"
    )
    default_config_filename = "cartographer.lua"
    default_rviz = os.path.join(
        default_config_dir, "cartographer.rviz"
    )


    # configs
    config_use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    config_map_dir = LaunchConfiguration("map_dir")
    config_map_name = LaunchConfiguration("map_name")
    config_map_save_late = LaunchConfiguration("map_save_late")
    config_use_rviz = LaunchConfiguration("use_rviz")
    config_use_navigation = LaunchConfiguration("use_navigation")
    config_config_dir = LaunchConfiguration("config_dir")
    config_config_filename = LaunchConfiguration("config_name")
    config_resolution = LaunchConfiguration('resolution')
    config_publish_period_sec = LaunchConfiguration('publish_period_sec')


    declare_map_dir = DeclareLaunchArgument(
        'map_dir', default_value=default_map_dir,
        description="マップ保存先のディレクトリを指定します。デフォルトはこのパッケージの map ディレクトリです。"
    )
    declare_map_name = DeclareLaunchArgument(
        'map_name', default_value=default_map_name,
        description="保存されるマップ名を指定します。"
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value="True",
        description="Rviz2 を起動します。"
    )
    declare_use_navigation = DeclareLaunchArgument(
        'use_navigation', default_value="True",
        description="Navigation を有効にします。"
    )
    declare_map_save_late = DeclareLaunchArgument(
        'map_save_late', default_value=defaulr_map_save_late,
        description="マップを自動保存する周期（/s）を指定します。"
    )
    declare_config_dir = DeclareLaunchArgument(
        'config_dir', default_value=default_config_dir,
        description=''
    )
    declare_config_name = DeclareLaunchArgument(
        'config_name', default_value=default_config_filename,
        description=''
    )
    declare_resolution = DeclareLaunchArgument(
        'resolution', default_value='0.025',
        description=''
    )
    declare_publish_period_sec = DeclareLaunchArgument(
        'publish_period_sec', default_value='1.0',
        description=''
    )

    ld.add_action(declare_map_dir)
    ld.add_action(declare_map_name)
    ld.add_action(declare_map_save_late)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_navigation)
    ld.add_action(declare_config_dir)
    ld.add_action(declare_config_name)
    ld.add_action(declare_resolution)
    ld.add_action(declare_publish_period_sec)


    # navigation
    launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('erasers_kachaka_navigation'),
                'launch', 'navigation_launch.py'
            )
        ]),
        launch_arguments={
            'use_map': 'False'
        }.items(),
        condition=IfCondition(config_use_navigation)
    )
    
    ld.add_action(launch_navigation)


    node_cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time':config_use_sim_time}],
        remappings=[
            ('/imu', f'/{NAMESPACE}/imu/imu'),
            ('/scan', f'/{NAMESPACE}/lidar/scan'),
            ('/odom', f'/{NAMESPACE}/odometry/odometry')
        ],
        arguments=[
            '-configuration_directory', config_config_dir,
            '-configuration_basename', config_config_filename,
            #'-load_state_filename',
        ]
    )
    node_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time':config_use_sim_time}],
        arguments=[
            '-resolution', config_resolution,
            '-publish_period_sec', config_publish_period_sec,
        ]
    )
    node_map_saver = Node(
        package='erasers_kachaka_cartographer',
        executable='map_saver',
        output='screen',
        parameters=[
            {'map_path': config_map_dir},
            {'map_name': config_map_name},
            {'save_late': config_map_save_late}
        ]
    )
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", default_rviz],
        condition=IfCondition(config_use_rviz)
    )
    
    ld.add_action(node_cartographer)
    ld.add_action(node_occupancy_grid_node)
    ld.add_action(node_map_saver)
    ld.add_action(node_rviz)


    return ld