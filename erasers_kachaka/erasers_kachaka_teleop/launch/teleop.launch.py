#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

import os

KACHAKA_NAME = os.environ.get('KACHAKA_NAME')

def generate_launch_description():
    ld = LaunchDescription()

    node_joy = Node(
        package="joy",
        executable="joy_node",
        namespace=KACHAKA_NAME
    )
    node_teleop = Node(
        package="erasers_kachaka_teleop",
        executable="teleop",
        namespace=KACHAKA_NAME
    )

    ld.add_action(node_joy)
    ld.add_action(node_teleop)
    
    return ld
