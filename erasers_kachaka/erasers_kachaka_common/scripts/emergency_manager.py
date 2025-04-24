#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from std_srvs.srv import Trigger
from sensor_msgs.msg import Joy

from kachaka_api import KachakaApiClient

from erasers_kachaka_common.tts import TTS

import os


KACHAKA_IP = os.environ.get('KACHAKA_IP')


class EmergencyButton(Node):
    def __init__(self):
        super().__init__('emergency_button')

        self.initialflag = False
        self.flag = False
        tts = TTS(self)
        self.say = tts.say

        self.sub = self.create_subscription(Joy, 'emergency/joy', self.cb, 10)
        self.cli = self.create_client(Trigger, 'emergency')

        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('emeregency manager is not running')
            raise RuntimeError('emeregency manager is not running')

        self.runtime()
    

    def cb(self, msg:Joy):
        self.flag = False
        if msg.buttons[0] == 0:
            self.flag = True
            if self.initialflag:
                self.get_logger().warn('EMERGENCY STOP!!!!')
        
        if msg.buttons[0] == 1:
            self.initialflag = True
    

    def runtime(self):
        self.get_logger().info('start emergency_button')

        future = None
        warning_said = False
        req = Trigger.Request()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.flag:
                if self.initialflag:
                    if future is None : self.say('停止')
                    future = self.cli.call_async(req)
                
                if not self.initialflag and not warning_said:
                    self.say('緊急停止ボタンがプッシュされています。一度リリースしてください。')
                    warning_said = True
            
            else: future = None


def emergency_button():
    rclpy.init()
    node = EmergencyButton()


class EmergencyManager(Node):
    def __init__(self):
        super().__init__('emergency_manager')

        self.get_logger().info('start emergency_manager')

        tts = TTS(self)
        self.say = tts.say

        self.kachaka = KachakaApiClient(f'{KACHAKA_IP}:26400')

        self.srv = self.create_service(Trigger, 'emergency', self.cb_srv)
    

    def cb_srv(self, req:Trigger.Request, res:Trigger.Response):
        self.kachaka.set_emergency_stop()

        res.message = "emergecy stop ! Please push power button when restart."
        res.success = True

        self.get_logger().error(res.message)

        return res


def emergency_manager():
    rclpy.init()

    node = EmergencyManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()