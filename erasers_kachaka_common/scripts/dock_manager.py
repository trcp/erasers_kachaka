#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from std_srvs.srv import SetBool

from kachaka_api import KachakaApiClient

import os


KACHAKA_IP = os.environ.get('KACHAKA_IP')


class DockManager(Node):
    def __init__(self):
        super().__init__('dock_manager')

        self.declare_parameter('kachaka_ip', '192.168.8.13')
        self.get_logger().info('start dock_manager')

        self.kachaka = KachakaApiClient(f'{self.get_parameter("kachaka_ip").value}:26400')

        self.srv = self.create_service(SetBool, 'docking_shelf', self.cb_srv)
    

    def cb_srv(self, req:SetBool.Request, res:SetBool.Response):
        if req.data:
            result = self.kachaka.dock_shelf()
            if result.success:
                res.message = "Success docking !"
                self.get_logger().info(res.message)
            else:
                res.message = "Failure docking. Error code: %d"%result.error_code
                self.get_logger().error(res.message)
        else:
            result = self.kachaka.undock_shelf()
            if result.success:
                res.message = "Success undocking !"
                self.get_logger().info(res.message)
            else:
                res.message = "Failure undocking. Error code: %d"%result.error_code
                self.get_logger().error(res.message)
        res.success = result.success
        return res


def dock_manager():
    rclpy.init()

    node = DockManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
