#!/usr/bin/env python3
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import shutil


def _prepare_runtime_cartographer_config(
    config_dir: str,
    config_name: str,
    frame_prefix: str,
    namespace: str,
) -> tuple[str, str]:
    if not frame_prefix:
        raise RuntimeError("frame_prefix is empty")
    if not frame_prefix.endswith('_'):
        raise RuntimeError("frame_prefix must end with '_'")
    if '/' in frame_prefix:
        raise RuntimeError("frame_prefix must not contain '/'")
    if any(c.isspace() for c in frame_prefix):
        raise RuntimeError("frame_prefix must not contain whitespace")

    if os.path.basename(config_name) != config_name:
        raise RuntimeError(f"config_name must be basename only: '{config_name}'")

    source_config_path = os.path.join(config_dir, config_name)
    if not os.path.exists(source_config_path):
        raise RuntimeError(f"Config file does not exist: '{source_config_path}'")

    runtime_dir = os.path.join('/tmp', f'cartographer_runtime_{namespace}')
    os.makedirs(runtime_dir, exist_ok=True)

    # Copy standard cartographer_ros and cartographer lua files
    search_dirs = [
        os.path.join(get_package_share_directory("cartographer_ros"), "configuration_files"),
    ]
    try:
        search_dirs.append(os.path.join(get_package_share_directory("cartographer"), "configuration_files"))
    except Exception:
        pass
    search_dirs.append('/opt/ros/humble/share/cartographer/configuration_files')

    for sdir in search_dirs:
        if os.path.exists(sdir):
            for fname in os.listdir(sdir):
                if fname.endswith('.lua'):
                    shutil.copy2(os.path.join(sdir, fname), os.path.join(runtime_dir, fname))

    # Copy and process custom lua files
    if os.path.exists(config_dir):
        for fname in os.listdir(config_dir):
            if fname.endswith('.lua'):
                src_path = os.path.join(config_dir, fname)
                dst_path = os.path.join(runtime_dir, fname)
                with open(src_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                content = content.replace('{prefix}', frame_prefix)
                if '{prefix}' in content:
                    raise RuntimeError(f"Unreplaced {{prefix}} remaining in {fname}")
                with open(dst_path, 'w', encoding='utf-8') as f:
                    f.write(content)

    return runtime_dir, config_name


def launch_setup(context, *args, **kwargs):
    namespace_val = LaunchConfiguration('namespace').perform(context)
    frame_prefix_val = LaunchConfiguration('frame_prefix').perform(context)
    use_map_save_val = LaunchConfiguration("use_map_save").perform(context)
    map_dir_val = LaunchConfiguration("map_dir").perform(context)
    map_name_val = LaunchConfiguration("map_name").perform(context)
    map_save_late_val = LaunchConfiguration("map_save_late")
    use_rviz_val = LaunchConfiguration("use_rviz").perform(context)
    use_sim_time_val = LaunchConfiguration("use_sim_time").perform(context)
    use_navigation_val = LaunchConfiguration("use_navigation").perform(context)
    config_dir_val = LaunchConfiguration("config_dir").perform(context)
    config_name_val = LaunchConfiguration("config_name").perform(context)
    resolution_val = LaunchConfiguration('resolution')
    publish_period_sec_val = LaunchConfiguration('publish_period_sec')

    runtime_dir, runtime_config_name = _prepare_runtime_cartographer_config(
        config_dir_val,
        config_name_val,
        frame_prefix_val,
        namespace_val,
    )

    default_params_file = os.path.join(
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
            'params_file': default_params_file,
        }.items(),
        condition=IfCondition(use_navigation_val)
    )

    remappings = [
        (['/', namespace_val, '/cartographer/imu'], ['/', namespace_val, '/imu/imu']),
        (['/', namespace_val, '/cartographer/scan'], ['/', namespace_val, '/lidar/scan']),
        (['/', namespace_val, '/cartographer/odom'], ['/', namespace_val, '/odometry/odometry']),
        ('/cartographer/map', ['/', namespace_val, '/cartographer/map']),
        ('/map', ['/', namespace_val, '/mapping/map']),
    ]

    node_cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time_val.lower() == 'true'}],
        remappings=remappings,
        arguments=[
            '-configuration_directory', runtime_dir,
            '-configuration_basename', runtime_config_name,
        ],
        namespace=[namespace_val, '/cartographer']
    )

    node_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time_val.lower() == 'true'}],
        arguments=[
            '-resolution', resolution_val,
            '-publish_period_sec', publish_period_sec_val,
        ],
        remappings=remappings,
        namespace=[namespace_val, '/cartographer']
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

    node_map_providor = Node(
        package='erasers_kachaka_cartographer',
        executable='map_providor',
        output='screen',
        remappings=remappings,
        namespace=[namespace_val, '/cartographer']
    )

    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", default_rviz],
        condition=IfCondition(use_rviz_val)
    )

    return [
        launch_navigation,
        node_cartographer,
        node_occupancy_grid_node,
        node_map_saver,
        node_map_providor,
        node_rviz,
    ]


def generate_launch_description():
    ld = LaunchDescription()

    default_map_dir = os.path.join(os.environ.get('HOME'), 'map')
    default_map_name = "test_field"
    default_map_save_rate = "5"
    default_config_dir = os.path.join(
        get_package_share_directory("erasers_kachaka_cartographer"), "config"
    )
    default_config_filename = "cartographer.lua"

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value=os.environ.get("KACHAKA_NAME", "er_kachaka"),
        description="Robot's name"
    )
    declare_frame_prefix = DeclareLaunchArgument(
        'frame_prefix', default_value=os.environ.get("KACHAKA_NAME", "er_kachaka") + "_",
        description="TF frame prefix"
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
    ld.add_action(declare_frame_prefix)
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

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld