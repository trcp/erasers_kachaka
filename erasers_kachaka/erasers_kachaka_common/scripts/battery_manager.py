#!/usr/bin/env python3
from std_msgs.msg import Float32
from sensor_msgs.msg import BatteryState
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange

from erasers_kachaka_common.tts import TTS

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import rclpy

import os
import time


QOS_PROFILE = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class BatteryManager(Node):
    def __init__(self):
        super().__init__("battery_manager")

        # parameters
        low_battery_level_descriptor = ParameterDescriptor(
            name="low_battery_level",
            type=rclpy.Parameter.Type.INTEGER.value,
            description="Warns when kachaka's battery level falls below a specified value.",
            integer_range=[IntegerRange(
                from_value=0,
                to_value=100,
                step=1
            )]
        )
        nofitication_late_descriptor = ParameterDescriptor(
            name="nofitication_late",
            type=rclpy.Parameter.Type.INTEGER.value,
            description="Defines the interval at which low-voltage warnings are notified.",
            integer_range=[IntegerRange(
                from_value=10,
                to_value=1000,
                step=1
            )]
        )
        
        self.declare_parameter("low_battery_level", 30, low_battery_level_descriptor)
        self.declare_parameter("nofitication_late",300, nofitication_late_descriptor)

        self.param_low_battery_level = self.get_parameter("low_battery_level").get_parameter_value().integer_value
        self.param_nofitication_late = self.get_parameter("nofitication_late").get_parameter_value().integer_value

        self.add_on_set_parameters_callback(self._params_cb)

        # common API setup
        tts = TTS(self)
        self.say = tts.say

        # declare first battery status flag
        self.declare_flag = False

        # create subscriber
        self.battery_sub = self.create_subscription(BatteryState, "robot_info/battery_state", self._cb, QOS_PROFILE)

        # create publisher
        self.battery_pub = self.create_publisher(Float32, "robot_info/battery", 10)
        self.battery = Float32()

        # initialize timer
        self.init_time = time.time()

    def _params_cb(self, params):
        for param in params:
            self.get_logger().info("Changed param %s : %d"%(param.name, param.value))
            if param.name == "low_battery_level":
                if param.value >= 0 and param.value <= 100:
                    self.param_low_battery_level = param.value
                else:
                    self.get_logger().warn("parameter error. can not set param %s to %d. this param range is 0 ~ 100"%(param.name, param.value))

            if param.name == "nofitication_late":
                if param.value >= 0 and param.value <= 100:
                    self.init_time = time.time()
                    self.param_nofitication_late = param.value
                else:
                    self.get_logger().warn("parameter error. can not set param %s to %d."%(param.name, param.value))

            else: continue

        return SetParametersResult(successful=True, reason="Changed Params")

    def _cb(self, msg):
        self.battery.data = round(msg.percentage * 100, 1)
        percentage = int(self.battery.data)
        charging = True if msg.power_supply_status==1 else False

        #print(self.param_nofitication_late, self.param_low_battery_level)
        if not self.declare_flag:
            self.get_logger().info(f"""
=========================
Battery: {percentage} %
Charging: {charging}
=========================
            """)
            self.declare_flag = True

        if time.time() - self.init_time > self.param_nofitication_late:
            self.init_time = time.time()

            if not charging and percentage < self.param_low_battery_level:
                text = "バッテリー残量が低下しています。残り%d％です"%percentage
                self.get_logger().warn(text)
                self.say(text, wait=False)

        self.battery_pub.publish(self.battery)

def main():
    rclpy.init()

    node = BatteryManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
