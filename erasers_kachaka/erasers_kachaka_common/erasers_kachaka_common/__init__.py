#!/usr/bin/env python3

from rclpy.node import Node

from tf2_ros import Buffer

from .tts import TTS
from .kachaka import *
from .navigator import Nav2Navigation

class Kachaka():
    def __init__(self, node:Node, buffer:Buffer):
        self.ros_node = node

        __tts = TTS(self.ros_node)
        self.say = __tts.say
        self.robot_stopper = RobotStopper(self.ros_node)

        try:
            self.navigation = Nav2Navigation(self.ros_node, 3.0, buffer)
        except RuntimeError:
            __node.get_logger().error('navigation is not running. can not use navigation command.')
