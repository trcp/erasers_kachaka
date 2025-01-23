#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from std_msgs.msg import String

from erasers_kachaka_common.tts import TTS

import os

NS = os.environ.get("KACHAKA_NAME")


class KachakaSpeakerSubscriber(Node):
    def __init__(self):
        super().__init__("kachaka_speaker_publisher")

        self.pub = self.create_subscription(String, "kachaka_speak", self.__cb, 10)

        self.tts = TTS(self)
    

    def __cb(self, msg:String):
        self.tts.say(msg.data, False)
        

def main():
    rclpy.init()

    node = KachakaSpeakerSubscriber()
    rclpy.spin(node)
