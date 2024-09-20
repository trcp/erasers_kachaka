#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    ## prefix
    map_save_dir = "/home/gai/erasers_ws/src/erasers_kachaka/erasers_kachaka/erasers_kachaka_navigation/map"
    ## config
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', 
        default=os.path.join(get_package_share_directory(
            'erasers_kachaka_navigation'
        ), 'cfg'))
    cartographer_conf_name = LaunchConfiguration('configuration_basename', default='cartographer.lua')
    resolution = LaunchConfiguration('resolution', default='0.025')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    ## arg
    declare_use_navigation = DeclareLaunchArgument(
        "use_navigation",
        default_value="false",
        description="enable Navigation2"
    )
    declare_use_map_saver = DeclareLaunchArgument(
        "use_map_saver",
        default_value="true",
        description="enable auto map saver"
    )
    declare_map_save_path = DeclareLaunchArgument(
        "map_save_path",
        default_value=map_save_dir,
        description="enable Navigation2"
    )
    declare_map_name = DeclareLaunchArgument(
        "map_name",
        default_value="map",
        description="Save map name"
    )
    declare_map_save_late = DeclareLaunchArgument(
        "map_save_late",
        default_value="5",
        description="Save map late"
    )

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        output='log',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ("/imu", "/er_kachaka/imu/imu"),
            ("/scan", "/er_kachaka/lidar/scan"),
            ("/odom", "/er_kachaka/odometry/odometry"),
        ],
        arguments=['-configuration_directory', cartographer_config_dir, '-configuration_basename', cartographer_conf_name]
        )
    occupancy = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])
    map_saver = Node(
        package="erasers_kachaka_navigation",
        executable="map_saver",
        name="auto_map_saver",
        parameters=[
            {"map_save_path" : LaunchConfiguration("map_save_path")},
            {"map_name" : LaunchConfiguration("map_name")},
            {"save_late" : LaunchConfiguration("map_save_late")}
        ],
        condition=IfCondition(LaunchConfiguration("use_map_saver"))
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            ThisLaunchFileDir(), "/navigation.launch.py"
        ]),
        launch_arguments={
            "slam_mode":LaunchConfiguration('slam_mode', default="true")
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_navigation"))
    )

    ld.add_action(declare_use_navigation)
    ld.add_action(declare_use_map_saver)
    ld.add_action(declare_map_save_path)
    ld.add_action(declare_map_name)
    ld.add_action(declare_map_save_late)
    ld.add_action(cartographer)
    ld.add_action(occupancy)
    ld.add_action(map_saver)
    ld.add_action(navigation_launch)
    
    return ld
