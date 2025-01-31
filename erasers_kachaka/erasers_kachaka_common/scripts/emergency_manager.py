#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from std_srvs.srv import Trigger

from kachaka_api import KachakaApiClient

from erasers_kachaka_common.tts import TTS

import os


KACHAKA_IP = os.environ.get('KACHAKA_IP')


class EmergencyManager(Node):
    def __init__(self):
        super().__init__('emergency_manager')

        self.get_logger().info('start emergency_manager.')

        tts = TTS(self)
        self.say = tts.say

        self.kachaka = KachakaApiClient(f'{KACHAKA_IP}:26400')

        self.srv = self.create_service(Trigger, 'emergency', self.cb_srv)
    

    def cb_srv(self, req:Trigger.Request, res:Trigger.Response):
        self.say("停止", True)

        self.kachaka.cancel_command()
        self.kachaka.set_emergency_stop()

        return res


def main():
    rclpy.init()

    node = EmergencyManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()