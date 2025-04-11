#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import depthai as dai
import cv2
import numpy as np

class OakNode(Node):
    def __init__(self):
        super().__init__("oak_rgb")

        pipeline = dai.Pipeline()
        cam_rgb = pipeline.createColorCamera()
        cam_rgb.setPreviewSize(320, 240)
        cam_rgb.setInterleaved(False)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)  
        cam_rgb.setFps(60)

        xout_rgb = pipeline.createXLinkOut()
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        # デバイス初期化とパイプラインの開始
        self.device = dai.Device(pipeline)

        # ROS2のImageメッセージを送信するパブリッシャーの作成
        self.image_pub = self.create_publisher(Image, 'oak/image_raw', 3)
        self.timer = self.create_timer(0.06, self.timer_callback)

    def timer_callback(self):
        # デバイスからフレームを取得
        in_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False).get()
        frame = in_rgb.getCvFrame()
        frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)

        # ROS2 Imageメッセージの作成と設定
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg() 
        msg.height = frame.shape[0]  # 画像の高さ
        msg.width = frame.shape[1]  # 画像の幅
        msg.encoding = 'bgr8'  # 画像のエンコーディング
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3  # 1行あたりのバイト数（RGBの3チャネル）
        msg.data = np.array(frame).tobytes()  # OpenCVフレームをバイト列に変換

        self.image_pub.publish(msg)

def main():
    rclpy.init()
    oakd_camera_node = OakNode()

    try:
        rclpy.spin(oakd_camera_node)
    except KeyboardInterrupt:
        pass
    oakd_camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
