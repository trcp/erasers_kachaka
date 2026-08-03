#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, IntegerRange
from std_msgs.msg import Int8

from kachaka_api import KachakaApiClient


class VolumeManager(Node):
    def __init__(self):
        super().__init__('volume_manager')

        volume_param_descriptor = ParameterDescriptor(name='volume',
                                                      type=rclpy.Parameter.Type.INTEGER.value,
                                                      description="Value of Kachaka speaker volume. KACHAKA: [0 ~ 10], KACHAKA PRO: [0 ~ 17]",
                                                      integer_range=[IntegerRange(from_value=0,
                                                                                  to_value=17,
                                                                                  step=1)])

        self.declare_parameter('kachaka_ip', '192.168.8.13')
        self.declare_parameter('volume', 10, volume_param_descriptor)

        self.param_kachaka_ip = self.get_parameter("kachaka_ip").value
        self.param_volume = self.get_parameter("volume").value

        self.add_on_set_parameters_callback(self._params_cb)

        self.kachaka = KachakaApiClient(f'{self.param_kachaka_ip}:26400')
        self.kachaka.set_speaker_volume(self.param_volume)

        self.sub = self.create_subscription(Int8, 'volume', self.cb, 10)
    

    def _params_cb(self, params):
        for param in params:
            self.get_logger().info("Changed param %s : %d"%(param.name, param.value))
            if param.name == 'volume':
                r = self.kachaka.set_speaker_volume(param.value)
                if not r.success:
                    self.get_logger().warn('This volume value is support Kachaka Pro only.')
                return SetParametersResult(successful=True, reason="Changed Params")


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
