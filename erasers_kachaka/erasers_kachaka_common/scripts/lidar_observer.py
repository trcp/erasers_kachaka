#!/usr/bin/env python3
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
import rclpy

from sensor_msgs.msg import LaserScan, BatteryState
from geometry_msgs.msg import Twist

from scipy.interpolate import interp1d
import numpy as np
import time


class LidarResampler(Node):
    def __init__(self):
        super().__init__('lidar_resampler')
        
        # パラメータ設定
        self.declare_parameter('target_points', 618)  # SLAMが要求する固定点数
        self.declare_parameter('input_scan_topic', '/kachaka/lidar/scan')
        self.declare_parameter('output_scan_topic', '/scan_resampled')

        self.target_points = self.get_parameter('target_points').value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # サブスクライバーとパブリッシャーの設定
        self.subscription = self.create_subscription(
            LaserScan,
            "input_scan",
            self.scan_callback,
            qos
        )
        
        self.publisher = self.create_publisher(
            LaserScan,
            "output_scan",
            qos
        )

        self.get_logger().info(f"Resampling LiDAR data to {self.target_points} points...")

    def scan_callback(self, msg: LaserScan):
        # オリジナルデータの角度配列を生成
        original_angles = np.arange(
            msg.angle_min,
            msg.angle_min + len(msg.ranges) * msg.angle_increment,
            msg.angle_increment
        )[:len(msg.ranges)]
        
        # 有効な距離データのみを抽出（0.0や無限大を除去）
        valid_mask = np.array(msg.ranges) > msg.range_min
        valid_angles = original_angles[valid_mask]
        valid_ranges = np.array(msg.ranges)[valid_mask]
        
        # 補間関数の作成（線形補間）
        interp_func = interp1d(
            valid_angles,
            valid_ranges,
            kind='linear',
            bounds_error=False,
            fill_value=msg.range_max  # 有効範囲外は最大距離で埋める
        )
        
        # ターゲット角度配列を生成（固定点数）
        target_angle_increment = (msg.angle_max - msg.angle_min) / self.target_points
        target_angles = np.linspace(
            msg.angle_min,
            msg.angle_max,
            self.target_points,
            endpoint=False
        )
        
        # 補間実行
        resampled_ranges = interp_func(target_angles)
        
        # 新しいLaserScanメッセージの作成
        new_scan = LaserScan()
        new_scan.header = msg.header
        new_scan.angle_min = msg.angle_min
        new_scan.angle_max = msg.angle_max
        new_scan.angle_increment = target_angle_increment
        new_scan.time_increment = msg.time_increment
        new_scan.scan_time = msg.scan_time
        new_scan.range_min = msg.range_min
        new_scan.range_max = msg.range_max
        new_scan.ranges = resampled_ranges.tolist()
        
        # 強度データがある場合の処理（オプション）
        if len(msg.intensities) == len(msg.ranges):
            interp_intensity = interp1d(
                valid_angles,
                np.array(msg.intensities)[valid_mask],
                kind='nearest',
                bounds_error=False,
                fill_value=0.0
            )
            new_scan.intensities = interp_intensity(target_angles).tolist()
        
        self.publisher.publish(new_scan)


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