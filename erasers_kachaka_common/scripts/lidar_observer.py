#!/usr/bin/env python3
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.node import Node
import rclpy
import math

from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist

import time


def determine_target_geometry(msg: LaserScan, target_points_param: int):
    if len(msg.ranges) < 2:
        return None
    if not (math.isfinite(msg.angle_min) and math.isfinite(msg.angle_max) and math.isfinite(msg.angle_increment)):
        return None
    if msg.angle_increment <= 0 or msg.angle_max <= msg.angle_min or msg.range_max <= msg.range_min:
        return None

    angular_span = msg.angle_max - msg.angle_min
    full_circle = abs(angular_span - 2.0 * math.pi) <= max(0.05, 2.0 * abs(msg.angle_increment))

    if target_points_param >= 2:
        target_points = target_points_param
    else:
        if full_circle:
            target_points = round(angular_span / msg.angle_increment)
        else:
            target_points = round(angular_span / msg.angle_increment) + 1

    if target_points < 2:
        return None

    if full_circle:
        output_angle_increment = angular_span / target_points
    else:
        output_angle_increment = angular_span / (target_points - 1)

    return {
        'target_points': target_points,
        'angle_min': msg.angle_min,
        'angle_max': msg.angle_max,
        'angular_span': angular_span,
        'angle_increment': output_angle_increment,
        'full_circle': full_circle,
        'input_count': len(msg.ranges),
    }


def process_scan(msg: LaserScan, locked_geo: dict):
    input_count = len(msg.ranges)
    target_points = locked_geo['target_points']

    if input_count < target_points:
        return None

    bin_min_ranges = [math.inf] * target_points

    if locked_geo['full_circle']:
        for i in range(input_count):
            val = msg.ranges[i]
            if math.isfinite(val) and msg.range_min <= val <= msg.range_max:
                bin_idx = int(math.floor((i / input_count) * target_points))
                if 0 <= bin_idx < target_points:
                    if val < bin_min_ranges[bin_idx]:
                        bin_min_ranges[bin_idx] = val
    else:
        denom = input_count - 1 if input_count > 1 else 1
        max_target_idx = target_points - 1
        for i in range(input_count):
            val = msg.ranges[i]
            if math.isfinite(val) and msg.range_min <= val <= msg.range_max:
                bin_idx = int(math.floor((i / denom) * max_target_idx + 0.5))
                bin_idx = min(max_target_idx, max(0, bin_idx))
                if val < bin_min_ranges[bin_idx]:
                    bin_min_ranges[bin_idx] = val

    out_msg = LaserScan()
    out_msg.header = msg.header
    out_msg.angle_min = locked_geo['angle_min']
    out_msg.angle_max = locked_geo['angle_max']
    out_msg.angle_increment = locked_geo['angle_increment']
    out_msg.time_increment = (msg.scan_time / target_points) if msg.scan_time > 0 else 0.0
    out_msg.scan_time = msg.scan_time
    out_msg.range_min = msg.range_min
    out_msg.range_max = msg.range_max
    out_msg.ranges = bin_min_ranges
    out_msg.intensities = []

    return out_msg


class LidarResampler(Node):
    def __init__(self):
        super().__init__('lidar_resampler')

        self.declare_parameter('target_points', 0)
        self.declare_parameter('input_scan_topic', 'lidar/scan_raw')
        self.declare_parameter('output_scan_topic', 'lidar/scan')

        self.target_points_param = self.get_parameter('target_points').value
        input_topic = self.get_parameter('input_scan_topic').value
        output_topic = self.get_parameter('output_scan_topic').value

        self.locked_geometry = None
        self.last_input_count = None

        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            qos_profile_sensor_data
        )

        out_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(
            LaserScan,
            output_topic,
            out_qos
        )

        self.get_logger().info(
            f"LidarResampler initialized: input_topic='{input_topic}', output_topic='{output_topic}'"
        )

    def scan_callback(self, msg: LaserScan):
        if self.locked_geometry is None:
            geo = determine_target_geometry(msg, self.target_points_param)
            if geo is None:
                self.get_logger().warn("Malformed initial scan received. Dropping scan.", throttle_duration_sec=5.0)
                return
            self.locked_geometry = geo
            self.get_logger().info(
                f"Locked geometry: input_count={geo['input_count']}, target_count={geo['target_points']}, "
                f"angle_min={geo['angle_min']:.4f}, angle_max={geo['angle_max']:.4f}, "
                f"output_angle_increment={geo['angle_increment']:.6f}, full_circle={geo['full_circle']}"
            )

        curr_count = len(msg.ranges)
        if self.last_input_count is not None and curr_count != self.last_input_count:
            self.get_logger().warn(
                f"Input range count changed from {self.last_input_count} to {curr_count}",
                throttle_duration_sec=5.0
            )
        self.last_input_count = curr_count

        out_msg = process_scan(msg, self.locked_geometry)
        if out_msg is None:
            self.get_logger().warn(
                f"Input count ({curr_count}) < target points ({self.locked_geometry['target_points']}). Dropping scan.",
                throttle_duration_sec=5.0
            )
            return

        self.publisher.publish(out_msg)


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

def lidar_resampler(args=None):
    rclpy.init(args=args)
    resampler = LidarResampler()
    rclpy.spin(resampler)
    resampler.destroy_node()
    rclpy.shutdown()


def lidar_observer():
    rclpy.init()

    node = LasesObserver()
    node.destroy_node()