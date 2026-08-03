#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    namespace_val = LaunchConfiguration('namespace').perform(context)
    frame_prefix_val = LaunchConfiguration('frame_prefix').perform(context)
    params_file_val = LaunchConfiguration('params_file').perform(context)
    use_sim_time_val = LaunchConfiguration('use_sim_time').perform(context)
    use_map_save_val = LaunchConfiguration('use_map_save').perform(context)
    map_dir_val = LaunchConfiguration('map_dir').perform(context)
    map_name_val = LaunchConfiguration('map_name').perform(context)
    map_save_late_val = LaunchConfiguration('map_save_late')
    use_rviz_val = LaunchConfiguration('use_rviz').perform(context)
    use_navigation_val = LaunchConfiguration('use_navigation').perform(context)

    replaced_params = ReplaceString(
        source_file=params_file_val,
        replacements={'{prefix}': frame_prefix_val}
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=replaced_params,
            root_key=[namespace_val, '/slam_toolbox'],
            param_rewrites={'use_sim_time': use_sim_time_val},
            convert_types=True),
        allow_substs=True
    )

    default_params_file_nav = os.path.join(
        get_package_share_directory('erasers_kachaka_navigation'),
        'params', 'navigation.yaml'
    )
    default_rviz = os.path.join(
        get_package_share_directory("erasers_kachaka_cartographer"),
        "config", "cartographer.rviz"
    )

    launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('erasers_kachaka_navigation'),
                'launch', 'navigation.launch.py'
            )
        ]),
        launch_arguments={
            'namespace': namespace_val,
            'frame_prefix': frame_prefix_val,
            'use_sim_time': use_sim_time_val,
            'use_map': 'False',
            'params_file': default_params_file_nav,
        }.items(),
        condition=IfCondition(use_navigation_val)
    )

    remappings = [
        ('/scan', ['/', namespace_val, '/lidar/scan']),
        ('/map', ['/', namespace_val, '/mapping/map']),
    ]

    node_slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[configured_params],
        remappings=remappings,
        namespace=[namespace_val, '/slam_toolbox']
    )

    node_map_saver = Node(
        package='erasers_kachaka_cartographer',
        executable='map_saver',
        output='screen',
        parameters=[
            {'map_path': map_dir_val},
            {'map_name': map_name_val},
            {'save_late': map_save_late_val}
        ],
        namespace=namespace_val,
        condition=IfCondition(use_map_save_val)
    )

    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", default_rviz],
        condition=IfCondition(use_rviz_val)
    )

    return [
        launch_navigation,
        node_slam_toolbox,
        node_map_saver,
        node_rviz,
    ]


def generate_launch_description():
    ld = LaunchDescription()

    default_params_file = os.path.join(
        get_package_share_directory('erasers_kachaka_cartographer'),
        'params', 'slam_toolbox.yaml'
    )
    default_map_dir = os.path.join(os.environ.get('HOME'), 'map')
    default_map_name = "test_field"
    default_map_save_rate = "5"

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value=os.environ.get('KACHAKA_NAME', 'er_kachaka'),
        description="Robot's name"
    )
    declare_frame_prefix = DeclareLaunchArgument(
        'frame_prefix',
        default_value=os.environ.get('KACHAKA_NAME', 'er_kachaka') + '_',
        description="TF frame prefix"
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description="Full path to the ROS2 parameters file to use"
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description="Use simulation (Gazebo) clock if true"
    )
    declare_use_map_save = DeclareLaunchArgument(
        'use_map_save',
        default_value="True",
        description="Enable automatic save map."
    )
    declare_map_dir = DeclareLaunchArgument(
        'map_dir',
        default_value=default_map_dir,
        description="Directory for save map."
    )
    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value=default_map_name,
        description="Name of map file."
    )
    declare_map_save_late = DeclareLaunchArgument(
        'map_save_late',
        default_value=default_map_save_rate,
        description="Define save map rate [sec]"
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value="False",
        description="Bringup RViz2"
    )
    declare_use_navigation = DeclareLaunchArgument(
        'use_navigation',
        default_value="false",
        description="Enable navigation"
    )

    ld.add_action(declare_namespace)
    ld.add_action(declare_frame_prefix)
    ld.add_action(declare_params_file)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_map_save)
    ld.add_action(declare_map_dir)
    ld.add_action(declare_map_name)
    ld.add_action(declare_map_save_late)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_navigation)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld