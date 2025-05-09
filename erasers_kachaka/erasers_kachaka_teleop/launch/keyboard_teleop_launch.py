#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

import os

KACHAKA_NAME = os.environ.get('KACHAKA_NAME')

def generate_launch_description():
    ld = LaunchDescription()

    node_keyboard_teleop = Node(
        package="erasers_kachaka_teleop",
        executable="keyboard_teleop",
        namespace=KACHAKA_NAME
    )

    ld.add_action(node_keyboard_teleop)
    
    return ld
