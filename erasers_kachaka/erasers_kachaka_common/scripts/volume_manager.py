#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from std_msgs.msg import Int8

from kachaka_api import KachakaApiClient


class VolumeManager(Node):
    def __init__(self):
        super().__init__('volume_manager')

        self.declare_parameter('kachaka_ip', '192.168.8.13')

        self.kachaka = KachakaApiClient(f'{self.get_parameter("kachaka_ip").value}:26400')

        self.sub = self.create_subscription(Int8, 'volume', self.cb, 10)


    def cb(self, msg):
        if msg.data >= 0 and msg.data <= 17:
            r = self.kachaka.set_speaker_volume(msg.data)
            if not r.success:
                self.get_logger().warn('This volume value is support Kachaka Pro only.')
        else: self.get_logger().warn('This volume value %d is out of range.'%msg.data)

def main():
    rclpy.init()
    node = VolumeManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
