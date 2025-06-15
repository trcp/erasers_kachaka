#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

import os


KACHAKA_NAME = os.environ.get('KACHAKA_NAME')


def generate_launch_description():
    ld = LaunchDescription()

    node_human_track = Node(
        package="erasers_kachaka_vision",
        executable="human_track",
        output="screen",
        emulate_tty=True,
        namespace=KACHAKA_NAME
    )

    ld.add_action(node_human_track)

    return ld
