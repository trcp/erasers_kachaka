#!/usr/bin/env python3
from erasers_kachaka_interfaces.srv import Speaker

from rclpy.node import Node
import rclpy

import traceback


class TTS(Node):
    def __init__(self, timeout=10):
        super().__init__("client_tts")

        # create client
        self._cli = self.create_client(Speaker, "/er_kachaka/tts")

        # wait server is avaiable
        while not self._cli.wait_for_service(timeout_sec=timeout):
            # rclpy.spin_once(self)
            self.get_logger().error("Service service_tts is not working !")

        # create field
        self.req = Speaker.Request()

    def say(
        self, text="引数テキストに発話させたい文字列を代入してください。", wait=True
    ):
        self.req.text = text
        future = self._cli.call_async(self.req)
        if wait:
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            return result.success
        else:
            return True

def __sample():
    rclpy.init()

    tts = TTS()

    tts.say(
        "あーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー"
    )


if __name__ == "__main__":
    __sample()