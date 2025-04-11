#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class OakReverse(Node):
    def __init__(self):
        super().__init__("oak_reverse")
        self.image_sub = self.create_subscription(Image, "/color/image", self.callback, 5)
        self.bridge = CvBridge()
        # ROS2のImageメッセージを送信するパブリッシャーの作成
        self.image_pub = self.create_publisher(Image, 'oak/image_reverse', 3)

    def callback(self,msg):
        # デバイスからフレームを取得
        cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        reversed_image = cv2.flip(cv_image, -1)
        msg_image = self.bridge.cv2_to_imgmsg(reversed_image, encoding="rgb8")
        self.image_pub.publish(msg_image)

def main():
    rclpy.init()
    oakd_camera_reverse = OakReverse()

    try:
        rclpy.spin(oakd_camera_reverse)
    except KeyboardInterrupt:
        pass
    oakd_camera_reverse.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
