#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os

NAMESPACE = "/er_kachaka"

# 起動時の発話メッセージを指定します。黙らせたい場合は空の文字列
DECLARE_BOOT_MESSAGE = "マニピュレーター、スタート！"

def generate_launch_description():
    ld = LaunchDescription()

    description = os.path.join(
        get_package_share_directory("erasers_kachaka_manipulation"),
        "urdf",
        "manipulation_description.urdf.xacro"
    )
    
    manipulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("nakalab_crane_x7_bringup"),
                "launch",
                "bringup.launch.py"
            )
        ]),
        launch_arguments={
            "description":description,
            "use_d435":"true"
        }.items()
    )

    ld.add_action(manipulation_launch)

    # launch message
    startup_sound_command = ExecuteProcess(
        cmd=[
            'ros2 service call %s/tts erasers_kachaka_interfaces/srv/Speaker "{text: %s}"'%(NAMESPACE, DECLARE_BOOT_MESSAGE)
        ],
        shell=True,
        output="own_log",
    )

    # load actions
    ld.add_action(startup_sound_command)

    return ld