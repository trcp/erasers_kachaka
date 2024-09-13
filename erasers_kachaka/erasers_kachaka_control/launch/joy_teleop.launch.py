from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    joy = Node(package="joy", executable="joy_node")

    teleop = Node(package="erasers_kachaka_control", executable="teleop_joy")

    ld.add_action(joy)
    ld.add_action(teleop)

    return ld
