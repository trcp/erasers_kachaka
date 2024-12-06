#!/usr/bin/env python3
"""
物体検出用 API
"""
# interfaces
from yolo_world_msgs.msg import *
from yolo_world_srvs.srv import *
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool

## TF
import tf2_ros
import tf2_geometry_msgs

## rclpy
from rclpy.node import Node
import rclpy

## general
from typing import List
import traceback

class ObjectDetect():
    def __init__(self, node:Node, 
                 tf_buffer:tf2_ros.Buffer=None,
                 timeout_sec:float=10.0,
                 ) -> None:
        """YoLo World で物体検出を有効にする API

        Args:
            node (Node): rclpy.node.Node オブジェクト
            tf_buffer (tf2_ros.Buffer, optional): TF Buffer オブジェクト. Defaults to None.
            timeout_sec (float, optional): サービス待機時間. Defaults to 10.0.
        """
        self.node = node
        self.tf_buffer = tf2_ros.Buffer() if tf_buffer is None else tf_buffer
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.interface_set_classes = SetClasses.Request()
        self.interface_set_predict = SetPredict.Request()
        self.interface_set_execute = SetBool.Request()
        self.object_poses_array:ObjectPoseArray = None
        self.object_images:ObjectImageArray = None
        self.object_poses = None

        self._cli_set_classes = self.node.create_client(SetClasses, '/yolo_world/set_classes')
        self._cli_set_predict = self.node.create_client(SetPredict, '/yolo_world/set_predict')
        self._cli_set_execute = self.node.create_client(SetBool, '/yolo_world/execute')

        self._sub_object_poses = self.node.create_subscription(ObjectPoseArray, '/yolo_world/object/poses', self._cb_object_poses, 10)

        while (
            not self._cli_set_classes.wait_for_service(timeout_sec=timeout_sec) or\
            not self._cli_set_predict.wait_for_service(timeout_sec=timeout_sec) or\
            not self._cli_set_execute.wait_for_service(timeout_sec=timeout_sec)
        ):
            self.node.get_logger().error('Docker yolo_world_ros2 is not running.')
        self.node.get_logger().info('Successful initialized !')
    
    # cb func
    def _cb_object_poses(self, msg:ObjectPoseArray) -> None:
        self.object_poses_array = msg
        self.object_poses = msg.poses
    
    def set_classes(self, classes:List[str]) -> bool:
        """検出したいオブジェクトを YoLo World に送信します.

        Args:
            classes (List[str]): 検出させたいオブジェクト名リスト

        Returns:
            bool: YoLo にクラスが送信されたかどうか
        """
        try:
            response:SetClasses.Response = None
            self.interface_set_classes.classes = classes
            future = self._cli_set_classes.call_async(self.interface_set_classes)

            self.node.get_logger().info(f'Send request classes: {classes}')
            while rclpy.ok() and response is None:
                rclpy.spin_once(self.node)
                response = future.result()
            self.node.get_logger().info(f'YoLo world serching classes: {classes}')
            return True
            
        except:
            self.node.get_logger().error(traceback.format_exc())
            return False
    
    def set_predict(self, predict:float) -> bool:
        """検出したいオブジェクトの確率を定義します.

        Args:
            predict (float): 検出させたいオブジェクト名リスト

        Returns:
            bool: YoLo に predict が送信されたかどうか
        """
        try:
            response:SetPredict.Response = None
            self.interface_set_predict.predict = predict
            future = self._cli_set_predict.call_async(self.interface_set_predict)

            self.node.get_logger().info(f'Send request predict: {predict}')
            while rclpy.ok() and response is None:
                rclpy.spin_once(self.node)
                response = future.result()
            self.node.get_logger().info(f'YoLo world serching object predict: {predict}')
            return True
            
        except:
            self.node.get_logger().error(traceback.format_exc())
            return False
    
    def set_execute(self, execute:bool) -> bool:
        """YoLo 検出を実行するかどうかを定義します.

        Args:
            execute (bool): True; 検出実行、False; 検出停止

        Returns:
            bool: YoLo に execute が送信されたかどうか
        """
        try:
            response:SetBool.Response = None
            self.interface_set_execute.data = execute
            future = self._cli_set_execute.call_async(self.interface_set_execute)

            while rclpy.ok() and response is None:
                rclpy.spin_once(self.node)
                response = future.result()
            self.node.get_logger().info(response.message)
            return True
            
        except:
            self.node.get_logger().error(traceback.format_exc())
            return False

    def get_object_infomation(self, classes:List[str],
                              predict:float=0.3,
                              ref_frame:str='crane_x7_mounting_plate_link'
                              ) -> list :
        print(self.node)
        try:
            self.set_classes(classes)
            self.set_predict(predict)
            self.set_execute(True)

            get_list_object_pose = ObjectPoseArray()
            get_object_pose = ObjectPose()

            object_pose:ObjectPose

            while rclpy.ok() and len(get_list_object_pose.poses) == 0:
                rclpy.spin_once(self.node)
                if self.object_poses is not None:
                    for object_pose in self.object_poses:
                        pose = object_pose.pose

                        ref_pose = PoseStamped()
                        ref_pose.header = self.object_poses_array.header
                        ref_pose.pose.position = pose
                        ref_pose.pose.orientation.w = 1.0

                        try:
                            get_object_pose = object_pose
                            get_object_pose.pose = self.tf_buffer.transform(ref_pose, ref_frame).pose.position
                            get_list_object_pose.poses.append(get_object_pose)
                            get_list_object_pose.header = ref_pose.header
                            get_list_object_pose.header.frame_id = ref_frame
                            
                        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                            self.node.get_logger().error(f'TF lookup failed: {e}')
            
            self.set_execute(False)
            return get_list_object_pose

        except:
            self.node.get_logger().error(traceback.format_exc())
            return []

if __name__ == "__main__":
    rclpy.init()
    node = Node("sample_object_detection")

    object_detect = ObjectDetect(node)
    object_pose = object_detect.get_object_infomation(classes=['red_cube'], predict=0.01)
    print(object_pose)