#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os


def generate_launch_description():
    ld = LaunchDescription()

    # plefix
    erasers_kachaka_description = get_package_share_directory('erasers_kachaka_description')


    # configures
    namespace = LaunchConfiguration('namespace')
    use_shelf = LaunchConfiguration('use_shelf')
    use_sim_time = LaunchConfiguration('use_sim_time')


    # arguments
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='er_kachaka',
        description='robot namespace'
    )
    declare_use_shelf = DeclareLaunchArgument(
        'use_shelf', default_value='false',
        description='Docking shelf'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='simulation time'
    )

    ld.add_action(declare_namespace)
    ld.add_action(declare_use_shelf)
    ld.add_action(declare_use_sim_time)


    # Xacro -> URDF
    xacro_file = os.path.join(
        erasers_kachaka_description, 'urdf', 'erasers_kachaka.urdf.xacro'
    )


    # node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'robot_description': Command([
                    'xacro ', xacro_file, ' ',
                    'use_shelf:=', use_shelf
                ]),
                'use_sim_time': use_sim_time
            }
        ]
    )

    ld.add_action(robot_state_publisher)


    return ld
