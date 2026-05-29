#!/usr/bin/env python3
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os


NAMESPACE = os.environ.get('KACHAKA_NAME')


def generate_launch_description():

    ld = LaunchDescription()

    # default variables
    default_params_file = os.path.join(
        get_package_share_directory('erasers_kachaka_navigation'),
        'params', 'navigation.yaml'
    )
    default_config_dir = os.path.join(
        get_package_share_directory("erasers_kachaka_cartographer"), "config"
    )

    # Launch Configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    use_map = LaunchConfiguration('use_map')
    map_dir = LaunchConfiguration('map_dir')
    map_name = LaunchConfiguration('map_name')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')

    # create params
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=[namespace, '/navigation'],
            param_rewrites={
                'use_sim_time': use_sim_time,
                'autostart': autostart
            },
            convert_types=True),
        allow_substs=True
    )

    remappings = [
        (['/', namespace, '/navigation/imu'], ['/', namespace, '/imu/imu']),
        (['/', namespace, '/navigation/scan'], ['/', namespace, '/lidar/scan']),
        (['/', namespace, '/navigation/map'], ['/', namespace, '/mapping/map']),
        (['/', namespace, '/navigation/goal_pose'], ['/', namespace, '/goal_pose']),
        ('/scan', ['/', namespace, '/lidar/scan']),
        ('/points', ['/', namespace, '/tof_camera/points']),
        ('/odom', ['/', namespace, '/odometry/odometry_reliable']),
    ]

    use_map_lifecycle_nodes = [
        'map_server',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]

    # declare arguments
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value=NAMESPACE,
        description="Robot's name"
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params_file,
        description='Full path for navigation yaml parameter file'
    )
    declare_use_map = DeclareLaunchArgument(
        'use_map', default_value='true',
        description='Enable import map'
    )
    declare_map_name = DeclareLaunchArgument(
        'map_dir', default_value=os.path.join(os.environ['HOME'], 'map'),
        description='Full path for map yaml file.'
    )
    declare_map_dir = DeclareLaunchArgument(
        'map_name', default_value='test_field',
        description='Full path for map yaml file.'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Enable autostart navigation'
    )
    declare_use_respawn = DeclareLaunchArgument(
        'use_respawn', default_value='true',
        description='Enable navigation reboot when occured error.'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value="False",
        description="If use rosbag"
    )

    ld.add_action(declare_namespace)
    ld.add_action(declare_params_file)
    ld.add_action(declare_use_map)
    ld.add_action(declare_map_dir)
    ld.add_action(declare_map_name)
    ld.add_action(declare_autostart)
    ld.add_action(declare_use_respawn)
    ld.add_action(declare_use_sim_time)

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    ld.add_action(stdout_linebuf_envvar)

    # Node Definitions
    node_emcl2 = Node(
        package="emcl2",
        executable="emcl2_node",
        name="emcl2",
        output="own_log",
        emulate_tty=True,
        parameters=[{'num_particles': 1000}],
        remappings=remappings
    )

    node_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        emulate_tty=True,
        condition=IfCondition(use_map),
        parameters=[{'yaml_filename': [map_dir, '/', map_name, '.yaml']}],
        remappings=remappings
    )

    node_nav2_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        emulate_tty=True,
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]
    )

    node_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        emulate_tty=True,
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", 'info'],
        remappings=remappings,
    )

    node_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        emulate_tty=True,
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", 'info'],
        remappings=remappings,
    )

    node_behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        emulate_tty=True,
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", 'info'],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]
    )

    node_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        emulate_tty=True,
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", 'info'],
        remappings=remappings,
    )

    node_waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        emulate_tty=True,
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", 'info'],
        remappings=remappings,
    )

    node_velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        emulate_tty=True,
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", 'info'],
        remappings=remappings + [
            ("cmd_vel", "cmd_vel_nav"),
            ("cmd_vel_smoothed", ['/', namespace, '/manual_control/cmd_vel'])
        ]
    )

    use_map_node_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        arguments=["--ros-args", "--log-level", 'info'],
        emulate_tty=True,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": use_map_lifecycle_nodes},
        ],
    )

    node_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        arguments=["--ros-args", "--log-level", 'info'],
        emulate_tty=True,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )

    node_odom_relay = Node(
        package='erasers_kachaka_navigation',
        executable='odom_relay_node',
        name='odom_relay_node',
        namespace=namespace,
        output='screen',
    )

    # Group Actions
    group_use_map_navigation = GroupAction(
        condition=IfCondition(use_map),
        actions=[
            PushRosNamespace([namespace, '/navigation']),
            node_emcl2,
            node_map_server,
            node_nav2_controller,
            node_smoother_server,
            node_planner_server,
            node_behavior_server,
            node_bt_navigator,
            node_waypoint_follower,
            node_velocity_smoother,
            use_map_node_lifecycle_manager,
        ]
    )

    group_navigation = GroupAction(
        condition=UnlessCondition(use_map),
        actions=[
            PushRosNamespace([namespace, '/navigation']),
            node_nav2_controller,
            node_smoother_server,
            node_planner_server,
            node_behavior_server,
            node_bt_navigator,
            node_waypoint_follower,
            node_velocity_smoother,
            node_lifecycle_manager,
        ]
    )

    ld.add_action(group_use_map_navigation)
    ld.add_action(group_navigation)
    ld.add_action(node_odom_relay)

    return ld