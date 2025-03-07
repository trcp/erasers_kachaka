#!/usr/bin/env python3
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
import rclpy

from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist

import time


class LasesObserver(Node):
    def __init__(self):
        super().__init__("laser_observer")

        self.flag = False
        self.charging = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, "lidar/scan", self.cb, qos)
        self.battery_sub = self.create_subscription(BatteryState, "robot_info/battery_state", self.battery_cb, qos)
        self.pub = self.create_publisher(Twist, "manual_control/cmd_vel", 10)

        self.main()
    

    def cb(self, msg):
        self.flag = True
    

    def battery_cb(self, msg:BatteryState):
        self.charging = not msg.power_supply_status - 1
    

    def main(self):
        twist = Twist()

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=5.0)

                if not self.flag and not self.charging:
                    self.get_logger().debug("Lidar reboot ...")
                    self.pub.publish(twist)

                self.flag = False
                time.sleep(1+30)
        
        except KeyboardInterrupt:
            pass


def main():
    rclpy.init()

    node = LasesObserver()
    node.destroy_node()