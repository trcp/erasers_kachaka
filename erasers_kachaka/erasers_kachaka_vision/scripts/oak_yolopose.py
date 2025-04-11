#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
import math

model = YOLO("yolo11n-pose.pt")

class OAKYOLO:
    def __init__(self, node):
        self.node = node
        self.image_pub = node.create_publisher(Image, "yolo_topic", 10)
        self.bridge = CvBridge()
        self.image_sub = node.create_subscription(Image, "oak/image_raw", self.callback, 10)

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

            results = model(bgr_image)
            anotated_frame = results[0].plot()
            keypoints = results[0].keypoints



            xys = keypoints.xy[0].tolist()
            img_msg = self.bridge.cv2_to_imgmsg(anotated_frame, "bgr8")

            self.image_pub.publish(img_msg)
            
            if len(xys) < 13:
                self.node.get_logger().warn(f"{len(xys)} keypoints found.")
                return
            
            etosL = math.sqrt((xys[7][0]-xys[5][0])**2  + (xys[7][1]-xys[5][1])**2)
            etosR = math.sqrt((xys[8][0]-xys[6][0])**2  + (xys[8][1]-xys[6][1])**2)
            htosL = math.sqrt((xys[11][0]-xys[5][0])**2  + (xys[11][1]-xys[5][1])**2)
            htosR = math.sqrt((xys[12][0]-xys[6][0])**2  + (xys[12][1]-xys[6][1])**2)

            #atan2はマイナスでも大丈夫
            angleL = math.degrees(math.atan2(etosL, htosL))
            angleR = math.degrees(math.atan2(etosR, htosR))



            if angleL > angleR:
                print("LLLLL")
            elif angleL < angleR:
                print("RRRRRRRRRRRRRR")
            else:
                print("MISS")

            time.sleep(1)
        except CvBridgeError as e:
            self.node.get_logger().error(f"CvBridge Error: {e}")
        except Exception as e:
            self.node.get_logger().error(f"Processing Error: {e}")

def main():
    rclpy.init()
    node = Node("oak_yolo")
    oak_disp = OAKYOLO(node)

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
