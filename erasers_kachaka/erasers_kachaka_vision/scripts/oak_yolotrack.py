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

model = YOLO("yolo11n-pose.pt")

class OAKYOLO:
    def __init__(self, node):
        self.node = node
        self.image_pub = node.create_publisher(Image, "yolo_topic", 10)
        self.human_pub = node.create_publisher(String, "human_pose", 10)
        self.bridge = CvBridge()
        self.image_sub = node.create_subscription(Image, "oak/image_reverse", self.callback, 10)

    def callback(self, msg):
        try:
            cx, cy = 0 , 0
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

            results = model(bgr_image)
            anotated_frame = results[0].plot()
            keypoints = results[0].keypoints
            xys = keypoints.xy[0].tolist()
            img_msg = self.bridge.cv2_to_imgmsg(anotated_frame, "bgr8")

            if len(xys) < 13:
                self.node.get_logger().warn(f"{len(xys)} keypoints found.")
                return

            self.image_pub.publish(img_msg)
            if 0 < xys[12][0] < 640 and 0 < xys[11][0] < 640:
                cx = (xys[12][0] + xys[11][0]) / 2
                cy = (xys[12][1] + xys[11][1]) / 2
                print(f"{cx},,,{cy}")

                if cx < 50 :
                    print("左左左")
                    human_dest = "-3"
                elif cx < 100 :
                    print("左左")
                    human_dest = "-2"
                elif cx < 130 :
                    print("左")
                    human_dest = "-1"
                elif cx < 170 :
                    print("○○○")
                    human_dest = "0"
                elif cx < 200 :
                    print("右")
                    human_dest = "1"
                elif cx < 250 :
                    print("右右")
                    human_dest = "2"
                elif cx >= 250 :
                    print("右右右")
                    human_dest = "3"
                else:
                    print("MISS")
                    human_dest = "9"
                human_msg = String()
                human_msg.data = human_dest
                self.human_pub.publish(human_msg)
                time.sleep(1)
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
