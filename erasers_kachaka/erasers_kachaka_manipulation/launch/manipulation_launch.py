#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from er_crane_x7_description.robot_description_loader import RobotDescriptionLoader
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ld = LaunchDescription()
    dl = RobotDescriptionLoader()


    prefix_semantic = os.path.join(
        get_package_share_directory("erasers_kachaka_manipulation"),
        "config", "crane_x7.srdf"
    )


    dl.port_name = "/dev/ttyUSB0"
    dl.use_d435 = "true"
    dl.gen_link = "false"
    dl.parent = "shelf_middle_plate_link"
    dl.manipulator_config_file_path = os.path.join(
        get_package_share_directory("crane_x7_control"),
        "config", "manipulator_config.yaml"
    )
    dl.manipulator_links_file_path = os.path.join(
        get_package_share_directory("crane_x7_control"),
        "config", "manipulator_links.csv"
    )
    dl.robot_description_path = os.path.join(
        get_package_share_directory("erasers_kachaka_description"),
        "urdf", "erasers_kachaka_with_manipulation_description.urdf.xacro"
    )
    description = dl.load()


    launch_manipulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("er_crane_x7_bringup"),
                "launch", "bringup.launch.py"
            )
        ]),
        launch_arguments={
            "description": description,
            "semantic": prefix_semantic
        }.items()
    )

    ld.add_action(launch_manipulation)

    action_move_state = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2 service call /move_groupstate er_crane_x7_srvs/srv/SetGoalstate"],
                shell=True,
                output='screen',
            )
        ]
    )

    ld.add_action(action_move_state)


    return ld