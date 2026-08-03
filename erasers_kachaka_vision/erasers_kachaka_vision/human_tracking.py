#!/usr/bin/env python3
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.node import Node
import rclpy

import tf2_ros
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

from message_filters import ApproximateTimeSynchronizer, Subscriber

from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import PoseStamped 
from kachaka_interfaces.msg import ObjectDetectionListStamped, ObjectDetection

from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2
import os


class HumanTrack(Node):
    def __init__(self):
        super().__init__('human_track')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/er_kachaka/human_pose/pose',
            10
        )

        sub_camera = Subscriber(self,
                                Image,
                                '/er_kachaka/front_camera/image_raw',
                                qos_profile=qos_profile
        )
        sub_camerainfo = Subscriber(self,
                                CameraInfo,
                                '/er_kachaka/front_camera/camera_info',
                                qos_profile=qos_profile
        )
        sub_lidar = Subscriber(self,
                                LaserScan,
                                '/er_kachaka/lidar/scan',
                                qos_profile=qos_profile
        )
        sub_objectdetect = Subscriber(self,
                                ObjectDetectionListStamped,
                                '/er_kachaka/object_detection/result',
                                qos_profile=qos_profile
        )

        self.ats = ApproximateTimeSynchronizer(
            [sub_camera, sub_camerainfo, sub_lidar, sub_objectdetect],
            queue_size=10,
            slop=0.2
        )
        self.ats.registerCallback(self.sync_callback)

        self.get_logger().info('Initialized Human Tracker...')


    def sync_callback(self, image:Image, camerainfo:CameraInfo, laserscan:LaserScan, objectdetectionstamped:ObjectDetectionListStamped):
        try:
            transform = self.tf_buffer.lookup_transform(
                laserscan.header.frame_id, camerainfo.header.frame_id, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Could not transform: {e}')
            return
        
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        K = np.array(camerainfo.k).reshape(3, 3)
        
        detectobject: ObjectDetection
        for detectobject in objectdetectionstamped.detection:
            # 人の検出情報を取得
            if detectobject.label != 0:
                continue

            roi = detectobject.roi
            x, y, w, h = roi.x_offset, roi.y_offset, roi.width, roi.height
            center_u = x + w // 2
            center_v = y + h // 2

            distance = self.get_distance_at_pixel(center_u, K, laserscan, transform)

            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.line(cv_image, (center_u - 10, center_v), (center_u + 10, center_v), (255, 255, 0), 1)
            cv2.line(cv_image, (center_u, center_v - 10), (center_u, center_v + 10), (255, 255, 0), 1)
            if distance is not None:
                dist_text = f"{distance:.2f} m"
            else:
                dist_text = "N/A"
            (text_w, text_h), baseline = cv2.getTextSize(dist_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(cv_image, (center_u - text_w//2 - 2, center_v + 15), (center_u + text_w//2 + 2, center_v + 15 + text_h + baseline), (0,0,0), -1)
            cv2.putText(cv_image, dist_text, (center_u - text_w//2, center_v + 15 + text_h), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
            if distance is not None:
                cx, cy = K[0, 2], K[1, 2]
                fx = K[0, 0]
                fy = K[1, 1]
                ray_camera_frame = np.array([(center_u - cx) / fx, (cy - cy) / fy, 1.0])
                quat = transform.transform.rotation
                rotation = R.from_quat([quat.x, quat.y, quat.z, quat.w])
                ray_lidar_frame = rotation.apply(ray_camera_frame)
                target_angle_rad = np.arctan2(ray_lidar_frame[1], ray_lidar_frame[0])

                pos_x = distance * np.cos(target_angle_rad)
                pos_y = distance * np.sin(target_angle_rad)
                pos_z = 0.0

                pose_msg = PoseStamped()
                pose_msg.header.stamp = laserscan.header.stamp
                pose_msg.header.frame_id = laserscan.header.frame_id

                pose_msg.pose.position.x = pos_x
                pose_msg.pose.position.y = pos_y
                pose_msg.pose.position.z = pos_z

                pose_msg.pose.orientation.x = 0.0
                pose_msg.pose.orientation.y = 0.0
                pose_msg.pose.orientation.z = 0.0
                pose_msg.pose.orientation.w = 1.0

                self.pose_pub.publish(pose_msg)
        
        cv2.waitKey(1)
        cv2.imshow('person pose', cv_image)
    

    def get_distance_at_pixel(self, u, camerainfo_k, laserscan, transform):
        cx, cy = camerainfo_k[0, 2], camerainfo_k[1, 2]
        fx, fy = camerainfo_k[0, 0], camerainfo_k[1, 1]

        ray_camera_frame = np.array([(u - cx) / fx, (cy - cy) / fy, 1.0])

        quat = transform.transform.rotation
        rotation = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        ray_lidar_frame = rotation.apply(ray_camera_frame)

        target_angle_rad = np.arctan2(ray_lidar_frame[1], ray_lidar_frame[0])

        if laserscan.angle_increment == 0.0:
            return None
        
        min_angle_diff = float('inf')
        best_index = -1
        for i in range(len(laserscan.ranges)):
            current_angle = laserscan.angle_min + i * laserscan.angle_increment
            angle_diff = abs(np.arctan2(np.sin(current_angle - target_angle_rad), np.cos(current_angle - target_angle_rad)))
            if angle_diff < min_angle_diff:
                min_angle_diff = angle_diff
                best_index = i

        if best_index != -1:
            distance = laserscan.ranges[best_index]
            if np.isinf(distance) or np.isnan(distance) or distance < laserscan.range_min or distance > laserscan.range_max:
                return None
            return distance
        return None


if __name__ == '__main__':
    rclpy.init()
    node = HumanTrack()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.shutdown()
