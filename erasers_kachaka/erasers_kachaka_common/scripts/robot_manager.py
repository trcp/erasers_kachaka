#!/usr/bin/env python3
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import BatteryState
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange

from erasers_kachaka_common.tts import TTS

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import rclpy

import os
import time
import traceback

import kachaka_api

KACHAKA_IP = os.getenv("KACHAKA_IP")
QOS_PROFILE = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

class EmergencyManager(Node):
    def __init__(self):
        super().__init__("emergency_manager")

        # Activate kachaka api
        self.kachaka = kachaka_api.KachakaApiClient(f"{KACHAKA_IP}:26400")
        # common API setup
        tts = TTS()
        self.say = tts.say

        # create service server
        self.srv = self.create_service(Trigger, "/er_kachaka/emergency", self.srv_cb)

    def srv_cb(self, req, res):
        self.say("停止", False)

        result = self.kachaka.set_emergency_stop()

        res.success = bool(result)
        res.message = "Emergency stop! Please push power button"

        self.get_logger().error(f"{res.message}")

        return res


def emergency_manager():
    rclpy.init()

    node = EmergencyManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


#################################################################################


class RTHManager(Node):
    def __init__(self):
        super().__init__("rth_manager")
        # Activate kachaka api
        self.kachaka = kachaka_api.KachakaApiClient(f"{KACHAKA_IP}:26400")
        # create service server
        self.srv = self.create_service(SetBool, "/er_kachaka/rth", self.srv_cb)
        # default is disabled
        self.kachaka.set_auto_homing_enabled(False)

    def srv_cb(self, req, res):
        result = self.kachaka.set_auto_homing_enabled(req.data)
        res.success = bool(result.success)

        if req.data:
            res.message = "Return To Home enabled"
        else:
            res.message = "Return To Home disabled"

        self.get_logger().info(f"{res.message}")

        return res


def rth_manager():
    rclpy.init()

    node = RTHManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


#################################################################################


class DockingManager(Node):
    def __init__(self):
        super().__init__("docking_manager")
        # Activate kachaka api
        self.kachaka = kachaka_api.KachakaApiClient(f"{KACHAKA_IP}:26400")
        # create service server
        self.srv = self.create_service(SetBool, "/er_kachaka/docking", self.srv_cb)

    def srv_cb(self, req, res):
        if req.data:
            result = self.kachaka.dock_shelf()
            message = "docking"
        else:
            result = self.kachaka.undock_shelf()
            message = "undocking"

        res.success = bool(result.success)

        if res.success:
            res.message = "Successflly " + message
        else:
            res.message = "Failure " + message

        self.get_logger().info(f"{res.message}")

        return res

def docking_manager():
    rclpy.init()

    node = DockingManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


#################################################################################

class BatteryManager(Node):
    def __init__(self):
        super().__init__("battery_manager")

        # parameters
        low_battery_level_descriptor = ParameterDescriptor(
            name="low_battery_level",
            type=rclpy.Parameter.Type.INTEGER,
            description="Warns when kachaka's battery level falls below a specified value.",
            integer_range=[IntegerRange(
                from_value=0,
                to_value=100,
                step=1
            )]
        )
        nofitication_late_descriptor = ParameterDescriptor(
            name="nofitication_late",
            type=rclpy.Parameter.Type.INTEGER,
            description="Defines the interval at which low-voltage warnings are notified.",
            integer_range=[IntegerRange(
                from_value=10,
                to_value=1000,
                step=1
            )]
        )
        
        self.declare_parameter("low_battery_level", 10, low_battery_level_descriptor)
        self.declare_parameter("nofitication_late",300, nofitication_late_descriptor)

        self.param_low_battery_level = self.get_parameter("low_battery_level").get_parameter_value().integer_value
        self.param_nofitication_late = self.get_parameter("nofitication_late").get_parameter_value().integer_value

        self.add_on_set_parameters_callback(self._params_cb)

        # Activate kachaka api
        self.kachaka = kachaka_api.KachakaApiClient(f"{KACHAKA_IP}:26400")
        # common API setup
        tts = TTS()
        self.say = tts.say

        # create subscriber
        self.battery_sub = self.create_subscription(BatteryState, "/er_kachaka/robot_info/battery_state", self._cb, QOS_PROFILE)

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
        percentage = int(msg.percentage * 100)
        charging = True if msg.power_supply_status==1 else False

        print(self.param_nofitication_late, self.param_low_battery_level)

        if time.time() - self.init_time > self.param_nofitication_late:
            self.init_time = time.time()

            if not charging and percentage < self.param_low_battery_level:
                text = "バッテリー残量が低下しています。残り%d％です"%percentage
                self.get_logger().warn(text)
                self.say(text, wait=False)

def battery_manager():
    rclpy.init()

    node = BatteryManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
