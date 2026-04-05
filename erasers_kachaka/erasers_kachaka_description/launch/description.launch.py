#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os


def generate_launch_description():
    ld = LaunchDescription()

    # default value
    default_robot_description = os.path.join(
        get_package_share_directory('erasers_kachaka_description'),
        'urdf', 'kachaka.urdf.xacro'
    )
    default_rviz_path = os.path.join(
        get_package_share_directory('erasers_kachaka_description'),
        'rviz', 'description.rviz'
    )


    # configures
    namespace = LaunchConfiguration('namespace')
    robot_description = LaunchConfiguration('robot_description')
    use_shelf = LaunchConfiguration('use_shelf')
    shelf_type = LaunchConfiguration('shelf_type')
    debug = LaunchConfiguration('debug')
    use_sim_time = LaunchConfiguration('use_sim_time')


    # arguments
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='er_kachaka',
        description='robot namespace'
    )
    declare_robot_descriptione = DeclareLaunchArgument(
        'robot_description', default_value=default_robot_description,
        description='Foll path for robot description'
    )
    declare_use_shelf = DeclareLaunchArgument(
        'use_shelf', default_value='false',
        description='Docking shelf'
    )
    declare_shelf_type = DeclareLaunchArgument(
        'shelf_type', default_value='2',
        description='Value of kachaka shelf layer'
    )
    declare_debug = DeclareLaunchArgument(
        'debug', default_value='false',
        description='Show robot model only'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='simulation time'
    )

    ld.add_action(declare_namespace)
    ld.add_action(declare_robot_descriptione)
    ld.add_action(declare_use_shelf)
    ld.add_action(declare_shelf_type)
    ld.add_action(declare_debug)
    ld.add_action(declare_use_sim_time)


    # Xacro -> URDF
    xacro_file = os.path.join(
        default_robot_description
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
                    'use_shelf:=', use_shelf, ' ',
                    'shelf_type:=', shelf_type
                ]),
                'use_sim_time': use_sim_time
            }
        ]
    )
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_path],
        emulate_tty=True,
        condition=IfCondition(debug)
    )

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz2)


    return ld
