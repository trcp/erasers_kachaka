from yolo_world_msgs.msg import ObjectImageArray, ObjectImage
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node
import rclpy

import cv2


class getObjectImage(Node):
    def __init__(self):
        super().__init__("test")

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.pub_depth_image = self.create_publisher(Image, 'objects/depth_image', 10)

        self.sub_object_images = Subscriber(self, ObjectImageArray, '/yolo_world/object/images', qos_profile=qos)
        self.sub_depth_images = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw', qos_profile=qos)
        self.sub_camerainfo_images = Subscriber(self, CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', qos_profile=qos)

        self.time_sync = ApproximateTimeSynchronizer(
            [self.sub_object_images, self.sub_depth_images, self.sub_camerainfo_images],
            10, 0.05, True
        )
        self.time_sync.registerCallback(self._cb)
        self.bridge = CvBridge()
    
    def _cb(self, object_images:ObjectImageArray, depth_imgmsg:Image, camera_ingo:CameraInfo):
        object:ObjectImage
        cv2.waitKey(1)

        depth_image = self.bridge.imgmsg_to_cv2(depth_imgmsg, '32FC1')
        for object in object_images.objects:
            image = self.bridge.imgmsg_to_cv2(object.image)
            cv2.imshow(f"{object.class_name}", image)

            depth_image = depth_image[object.pose1.y:object.pose2.y, object.pose1.x:object.pose2.x]
            cv2.imshow(f"depth: {object.class_name}", depth_image)
        
        depthimg_msg = self.bridge.cv2_to_imgmsg(depth_image, '32FC1')
        self.pub_depth_image.publish(depthimg_msg)

if __name__ == "__main__":
    rclpy.init()
    node = getObjectImage()
    rclpy.spin(node)
