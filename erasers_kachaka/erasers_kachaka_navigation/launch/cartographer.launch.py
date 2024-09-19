#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    ## config
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', 
        default=os.path.join(get_package_share_directory(
            'erasers_kachaka_navigation'
        ), 'cfg'))
    cartographer_conf_name = LaunchConfiguration('configuration_basename', default='cartographer.lua')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        output='log',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ("/imu", "/er_kachaka/imu/imu"),
            ("/scan", "/er_kachaka/lidar/scan"),
        ],
        arguments=['-configuration_directory', cartographer_config_dir, '-configuration_basename', cartographer_conf_name]
        )
    occupancy = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    ld.add_action(cartographer)
    ld.add_action(occupancy)
    
    return ld
