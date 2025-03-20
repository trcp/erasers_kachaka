#!/usr/bin/env python3
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os


NAMESPACE = os.environ.get('KACHAKA_NAME')


def generate_launch_description():

    ld = LaunchDescription()


    # default valiables
    default_map = '~/map/test_field.yaml'
    default_params_file = os.path.join(
        get_package_share_directory('erasers_kachaka_navigation'),
        'params', 'navigation.yaml'
    )
    default_config_dir = os.path.join(
        get_package_share_directory("erasers_kachaka_cartographer"),"config"
    )
    default_rviz = os.path.join(
        default_config_dir, "navigation.rviz"
    )

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/scan', f'/{NAMESPACE}/lidar/scan'),
        ('/local_costmap/scan', f'/{NAMESPACE}/lidar/scan'),
        ('/odom', f'/{NAMESPACE}/odometry/odometry'),
        ('/cmd_vel', f'/{NAMESPACE}/manual_control/cmd_vel'),
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


    # configs
    config_use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    config_params_file = LaunchConfiguration('params_file')
    config_use_map = LaunchConfiguration('use_map')
    config_map = LaunchConfiguration('map')
    config_autostart = LaunchConfiguration('autostart')
    config_use_respawn = LaunchConfiguration('use_respawn')


    # create params
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=config_params_file,
            param_rewrites={
                'use_sim_time': config_use_sim_time,
                'autostart': config_autostart
            },
            convert_types=True),
        allow_substs=True
    )


    # declare arguments
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params_file,
        description='Navigation パラメータYAMLファイルを指定します。'
    )
    declare_use_map = DeclareLaunchArgument(
        'use_map', default_value='true',
        description='マップを使用します。'
    )
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='ナビゲーションで使用するマップYAMLファイルを指定します。'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Nav2 Stack の自動起動を有効にします。'
    )
    declare_use_resparn = DeclareLaunchArgument(
        'use_respawn', default_value='true',
        description='Nav2 Stack の再起動を有効にします。'
    )
    ld.add_action(declare_params_file)
    ld.add_action(declare_use_map)
    ld.add_action(declare_map)
    ld.add_action(declare_autostart)
    ld.add_action(declare_use_resparn)


    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    ld.add_action(stdout_linebuf_envvar)

    result = PythonExpression(['not', config_use_map])
    print(result)


    # Node
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", default_rviz],
    )
    node_emcl2 = Node(
        package="emcl2",
        executable="emcl2_node",
        name="emcl2",
        output="own_log",
        remappings=remappings
    )
    node_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        condition=IfCondition(config_use_map),
        parameters=[
            {'yaml_filename': config_map}
        ]
    )
    node_nav2_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]
    )
    node_smoother_server =  Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        respawn=config_use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", 'info'],
        remappings=remappings,
    )
    node_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        respawn=config_use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", 'info'],
        remappings=remappings,
    )
    node_behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        respawn=config_use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", 'info'],
        remappings=remappings,
    )
    node_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        respawn=config_use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", 'info'],
        remappings=remappings,
    )
    node_waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        respawn=config_use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", 'info'],
        remappings=remappings,
    )
    node_velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        respawn=config_use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", 'info'],
        remappings=remappings
        + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
    )
    use_map_node_lifecycle_manager =  Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        arguments=["--ros-args", "--log-level", 'info'],
        parameters=[
            {"use_sim_time": config_use_sim_time},
            {"autostart": config_autostart},
            {"node_names": use_map_lifecycle_nodes},
        ],
    )
    node_lifecycle_manager =  Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        arguments=["--ros-args", "--log-level", 'info'],
        parameters=[
            {"use_sim_time": config_use_sim_time},
            {"autostart": config_autostart},
            {"node_names": lifecycle_nodes},
        ],
    )


    group_use_map_navigation = GroupAction(
        condition=IfCondition(config_use_map),
        actions=[
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
        condition=UnlessCondition(config_use_map),
        actions=[
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

    ld.add_action(node_rviz)
    ld.add_action(node_emcl2)
    ld.add_action(group_use_map_navigation)
    ld.add_action(group_navigation)
    

    return ld
