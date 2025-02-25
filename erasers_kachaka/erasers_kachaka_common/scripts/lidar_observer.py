#!/usr/bin/env python3
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
import rclpy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LasesObserver(Node):
    def __init__(self):
        super().__init__("laser_observer")

        self.flag = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, "lidar/scan", self.cb, qos)
        self.pub = self.create_publisher(Twist, "manual_control/cmd_vel", 10)

        self.main()
    

    def cb(self, msg):
        self.flag = True
    

    def main(self):
        twist = Twist()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.25)

            if not self.flag:
                #self.get_logger().info("Lidar reboot ...")
                self.pub.publish(twist)

            self.flag = False


def main():
    rclpy.init()

    node = LasesObserver()
    node.destroy_node()