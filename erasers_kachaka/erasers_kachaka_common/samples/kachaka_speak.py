#!/usr/bin/env python3

# ROS modules
from rclpy.node import Node
import rclpy

# Speaker API
from erasers_kachaka_common.tts import TTS


# initialize rclpy
rclpy.init()

# create ROS2 node
node = Node("sample_kachaka_speak")

# initialize TTS
tts = TTS(node)

# speak!
text = "Hello World!"
print(tts.say(text))