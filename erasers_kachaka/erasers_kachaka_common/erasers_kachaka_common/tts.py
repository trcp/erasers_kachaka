#!/usr/bin/env python3
from kachaka_interfaces.action import ExecKachakaCommand
from kachaka_interfaces.msg import KachakaCommand

# rcl
from rclpy.action import ActionClient
from rclpy.node import Node
import rclpy

import os


NS = os.environ.get("KACHAKA_NAME")

class TTS():
    def __init__(self, node:Node, wait_time=10):
        self.__node = node

        self.__action_client = ActionClient(self.__node, ExecKachakaCommand, "/%s/kachaka_command/execute"%NS)

        if not self.__action_client.wait_for_server(timeout_sec=wait_time):
            self.__node.get_logger().fatal("May be KACHAKA is not running ...")
        
        self.__node.get_logger().info("TTS start")
        # create command and action field
        self.__cmd = KachakaCommand()
        self.__cmd.command_type = KachakaCommand.SPEAK_COMMAND

        self.__goal_msg = ExecKachakaCommand.Goal()
    
    def __cb_response(self, future):
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        print(result_future)


    def say(self, text:str, wait=True):
        self.__cmd.speak_command_text = text
        self.__goal_msg.kachaka_command = self.__cmd
        
        future = self.__action_client.send_goal_async(self.__goal_msg)
        future.add_done_callback(self.__cb_response)
        
        if wait:
            rclpy.spin_until_future_complete(self.__node, future)