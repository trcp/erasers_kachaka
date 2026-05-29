# [Canvas]修正された cartographer 起動 launch ファイル
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ld = LaunchDescription()

    # 各コンフィグのデフォルト値
    default_map_dir = os.path.join(os.environ.get('HOME'), 'map')
    default_map_name = "test_field"
    default_map_save_rate = "5"
    default_config_dir = os.path.join(
        get_package_share_directory("erasers_kachaka_cartographer"), "config"
    )
    default_config_filename = "cartographer.lua"
    default_rviz = os.path.join(
        default_config_dir, "cartographer.rviz"
    )
    default_params_file = os.path.join(
        get_package_share_directory('erasers_kachaka_navigation'),
        'params', 'navigation.yaml'
    )

    # Launch Configurations
    namespace = LaunchConfiguration('namespace')
    use_map_save = LaunchConfiguration("use_map_save")
    map_dir = LaunchConfiguration("map_dir")
    map_name = LaunchConfiguration("map_name")
    map_save_late = LaunchConfiguration("map_save_late")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_navigation = LaunchConfiguration("use_navigation")
    config_dir = LaunchConfiguration("config_dir")
    config_filename = LaunchConfiguration("config_name")
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    # Declare Launch Arguments
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='er_kachaka',
        description="Robot's name"
    )
    declare_use_map_save = DeclareLaunchArgument(
        'use_map_save', default_value="True",
        description="Enable automatic save map."
    )
    declare_map_dir = DeclareLaunchArgument(
        'map_dir', default_value=default_map_dir,
        description="Directory for save map."
    )
    declare_map_name = DeclareLaunchArgument(
        'map_name', default_value=default_map_name,
        description="Name of map file."
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value="False",
        description="Bringup RViz2"
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value="False",
        description="If use rosbag"
    )
    declare_use_navigation = DeclareLaunchArgument(
        'use_navigation', default_value="false",
        description="Enable navigation"
    )
    declare_map_save_late = DeclareLaunchArgument(
        'map_save_late', default_value=default_map_save_rate,
        description="Define save map rate [sec]"
    )
    declare_config_dir = DeclareLaunchArgument(
        'config_dir', default_value=default_config_dir,
        description='Full path for Cartographer config directory'
    )
    declare_config_name = DeclareLaunchArgument(
        'config_name', default_value=default_config_filename,
        description='Cartographer config .lua file name.'
    )
    declare_resolution = DeclareLaunchArgument(
        'resolution', default_value='0.025',
        description='Map resolution'
    )
    declare_publish_period_sec = DeclareLaunchArgument(
        'publish_period_sec', default_value='1.0',
        description='Map update period.'
    )

    ld.add_action(declare_namespace)
    ld.add_action(declare_use_map_save)
    ld.add_action(declare_map_dir)
    ld.add_action(declare_map_name)
    ld.add_action(declare_map_save_late)
    ld.add_action(declare_use_sim_time)
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
            'use_map': 'False',
            'params_file': default_params_file,
            'use_rviz': 'False'
        }.items(),
        condition=IfCondition(use_navigation)
    )
    
    ld.add_action(launch_navigation)

    remappings = [
        (['/', namespace, '/cartographer/imu'], ['/', namespace, '/imu/imu']),
        (['/', namespace, '/cartographer/scan'], ['/', namespace, '/lidar/scan']),
        (['/', namespace, '/cartographer/odom'], ['/', namespace, '/odometry/odometry']),
        ('/cartographer/map', ['/', namespace, '/cartographer/map']),
        ('/map', ['/', namespace, '/mapping/map']),
    ]

    node_cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time},
                    {'frame_prefix': ''}],
        remappings=remappings,
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', config_filename,
        ],
        namespace=[namespace, '/cartographer']
    )
    
    node_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', resolution,
            '-publish_period_sec', publish_period_sec,
        ],
        remappings=remappings,
        namespace=[namespace, '/cartographer']
    )
    
    node_map_saver = Node(
        package='erasers_kachaka_cartographer',
        executable='map_saver',
        output='screen',
        parameters=[
            {'map_path': map_dir},
            {'map_name': map_name},
            {'save_late': map_save_late}
        ],
        namespace=namespace,
        condition=IfCondition(use_map_save)
    )
    
    node_map_providor = Node(
        package='erasers_kachaka_cartographer',
        executable='map_providor',
        output='screen',
        remappings=remappings,
        namespace=[namespace, '/cartographer']
    )
    
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", default_rviz],
        condition=IfCondition(use_rviz)
    )
    
    ld.add_action(node_cartographer)
    ld.add_action(node_occupancy_grid_node)
    ld.add_action(node_map_saver)
    ld.add_action(node_map_providor)
    ld.add_action(node_rviz)

    return ld