#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO
import math

#編集中
model = YOLO("yolo11n-seg.pt")

class OAKYOLO:
    def __init__(self, node):
        self.node = node
        self.image_pub = node.create_publisher(Image, "yolo_topic", 10)
        self.seg_pub = node.create_publisher(String, "yolo_seg", 10)
        self.bridge = CvBridge()
        self.image_sub = node.create_subscription(Image, "oak/image_reverse", self.callback, 10)

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

            results = model(bgr_image)
            result = results[0]
            frame = result.plot()
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(img_msg)

        except CvBridgeError as e:
            self.node.get_logger().error(f"CvBridge Error: {e}")
        except Exception as e:
            self.node.get_logger().error(f"Processing Error: {e}")

def main():
    rclpy.init()
    node = Node("oak_yolo")
    xtion_disp = OAKYOLO(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("SHUT DOWN")
        cv2.destroyAllWindows()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
