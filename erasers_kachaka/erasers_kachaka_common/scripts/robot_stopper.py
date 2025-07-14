#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

class RobotStopper(Node):
    def __init__(self):
        super().__init__('robot_stopper')

        self.pub = self.create_publisher(Twist, '/er_kachaka/manual_control/cmd_vel', 10)

        srv = self.create_service(SetBool, '/er_kachaka/robot_stopper', self.cb)

        timer = self.create_timer(0.1, self.robot_stopper)

        # stopper flag
        self.stop = False

        self.twist = Twist()
        self.inverse = False # 前後移動判定


    def cb(self, req, res):
        self.stop = req.data

        res.success = True
        res.message = 'robot stopper is enabled' if self.stop else 'robot stopper is disabled'

        return res


    def robot_stopper(self):
        if self.inverse:
            self.twist.linear.x = 0.3
            self.inverse = False
        else:
            self.twist.linear.x = -0.3
            self.inverse = True

        if self.stop:
            self.pub.publish(self.twist)

def main():
    rclpy.init()
    node = RobotStopper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
