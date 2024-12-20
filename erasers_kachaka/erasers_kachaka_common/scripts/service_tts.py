#!/usr/bin/env python3
from erasers_kachaka_interfaces.srv import Speaker
from std_msgs.msg import String

from rclpy.node import Node
import rclpy

import os
import traceback

import kachaka_api

import grpc

KACHAKA_IP = os.getenv("KACHAKA_IP")


class TTS(Node):
    def __init__(self):
        super().__init__("service_tts")

        # Activate kachaka api
        self.kachaka = kachaka_api.KachakaApiClient(f"{KACHAKA_IP}:26400")

        # create service server
        self.srv = self.create_service(Speaker, "tts", self.srv_cb)

        # create subscriber
        self.sub = self.create_subscription(String, "tts", self.sub_cb, 10)

    def srv_cb(self, req, res):
        try:
            print(f"get request: {req.text}")
            result = self.kachaka.speak(req.text)
            res.success = result.success
        except grpc.RpcError as e:
            self.get_logger().error(e)
            res.success = False
        finally:
            return res

    def sub_cb(self, msg):
        text = msg.data

        self.get_logger().info(f"Get string message: {text}")
        self.kachaka.speak(text)

def main():
    rclpy.init()

    node = TTS()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
