#!/usr/bin/env python3
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import rclpy

from message_filters import Subscriber, ApproximateTimeSynchronizer
import message_filters

from cv_bridge import CvBridge, CvBridgeError
import cv2

from sensor_msgs.msg import Image
from kachaka_interfaces.msg import ObjectDetectionListStamped

CB =CvBridge()

class ObjectDetectionPublisher(Node):
    def __init__(self):
        super().__init__("object_detect_publisher")

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
            )

        self.frontcamera_sub = Subscriber(
            self, Image,
            "image_raw",
            qos_profile=qos
        )
        self.detectresult_sub = Subscriber(
            self, ObjectDetectionListStamped,
            "result",
            qos_profile=qos
        )

        self.time_sync = ApproximateTimeSynchronizer(
            [self.frontcamera_sub, self.detectresult_sub],
            10, 0.05
        )
        self.time_sync.registerCallback(self._cb)

        self.get_logger().info("detection publisher start")

    def _cb(self, image, result):
        try:
            image = CB.imgmsg_to_cv2(image)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e :
                self.get_logger().error(e)

        for i, r in enumerate(result.detection):
            if r.label == 0: c = (255,0,0); l = "Person"
            if r.label == 1: c = (0,255,0); l = "Shelf"
            if r.label == 2: c = (0,0,255); l = "Charger"
            if r.label == 3: c = (255,0,255); l = "Door"
            
            cv2.rectangle(
                image,
                (r.roi.x_offset, r.roi.y_offset),
                (r.roi.x_offset + r.roi.width, r.roi.y_offset + r.roi.height),
                c,
                2
            )

            score = r.score
            str(int(score * 100))
            
            
            if r.roi.y_offset < 20:
                cv2.putText(
                    image,
                    f"{l} :{int(score * 100)}",
                    (r.roi.x_offset, r.roi.y_offset + r.roi.height + 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    c,
                    2
                 )
            else:
                cv2.putText(
                    image,
                    f"{l} :{int(score * 100)}",
                    (r.roi.x_offset, r.roi.y_offset - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    c,
                    2
                 )

        cv2.imshow("test", image)
        cv2.waitKey(1)

def main():
    rclpy.init()

    node = ObjectDetectionPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("done")
    finally:
        node.destroy_node()
