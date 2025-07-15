#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from std_srvs.srv import SetBool

import os


NS = os.environ.get("KACHAKA_NAME") 


class RobotStopper():
    """
    Kachaka を前後に微速移動させ、そのばでロボットを固定させます。
    ロボットと正常に接続できなかった場合 RuntimeError がスローされます。

    Args:
        node: rclpy.node.Node : rclpy Node オブジェクト。代入必須です。
    """
    def __init__(self, node:Node):
        self.__node = node

        self.__client = self.__node.create_client(SetBool, f'/{NS}/robot_stopper')
        while not self.__client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().error('May be KACHAKA is not running ...')
            raise RuntimeError('May be KACHAKA is not running ...')

    def __send_req(self, req:SetBool.Request):
        future = self.__client.call_async(req)

        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10.0)
        if future.done():
            res = future.result()
            self.__node.get_logger().info(res.message)
            return True
        else:
            return False
    
    
    def enable(self):
        """_summary_
        カチャカをその場で固定させます。

        Returns:
            bool: ストッパーが動作すると True。なにか問題が発生すると False をスローします。
        """
        req = SetBool.Request()
        req.data = True

        return self.__send_req(req)


    def disable(self):
        """
        カチャカの固定を解除します。

        Returns:
            bool: ストッパーが停止すると True。なにか問題が発生すると False をスローします。
        """
        req = SetBool.Request()
        req.data = False

        return self.__send_req(req)
