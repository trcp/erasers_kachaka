#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

class PointPublisher(Node):
    def __init__(self):
        super().__init__('point_and_marker_publisher')

        self.declare_parameter('target_frame', 'container_kachaka/base_link')
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        self.point_pub = self.create_publisher(PointStamped, '/container_point', 10)
        self.marker_pub = self.create_publisher(Marker, '/container_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_point_and_marker)

    def publish_point_and_marker(self):
        point_msg = PointStamped()
        point_msg.header.frame_id = self.target_frame
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.point.x = 1.0
        point_msg.point.y = 0.0
        point_msg.point.z = 0.5

        marker_msg = Marker()
        marker_msg.header.frame_id = self.target_frame
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.ns = "point_marker"
        marker_msg.id = 0
        marker_msg.type = Marker.SPHERE
        marker_msg.action = Marker.ADD
        marker_msg.pose.position.x = point_msg.point.x
        marker_msg.pose.position.y = point_msg.point.y
        marker_msg.pose.position.z = point_msg.point.z
        marker_msg.pose.orientation.w = 1.0
        marker_msg.scale.x = 0.1
        marker_msg.scale.y = 0.1
        marker_msg.scale.z = 0.1
        marker_msg.color.r = 1.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 0.0
        marker_msg.color.a = 1.0

        self.point_pub.publish(point_msg)
        self.marker_pub.publish(marker_msg)

        self.get_logger().info(f'Published Point and Marker in frame: {self.target_frame}')


def main(args=None):
    rclpy.init(args=args)
    node = PointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
