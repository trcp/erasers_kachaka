#!/usr/bin/env python3

## interfaces
from yolo_world_msgs.msg import ObjectPoseArray
from yolo_world_srvs.srv import SetClasses
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool

## TF
import tf2_ros
import tf2_geometry_msgs

## rclpy
from rclpy.node import Node
import rclpy

## general
import traceback

class GetObjectPose():
    def __init__(self, node:Node, timeout_sec:float=20.0, buffer:tf2_ros.Buffer=None) ->None:
        self.node = node
        self.tf_buffer = tf2_ros.Buffer() if buffer is None else buffer
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.set_classes_interface = SetClasses.Request()
        self.execute_interface = SetBool.Request()
        self._obj_poses: ObjectPoseArray = None

        # create clients
        self.execute_detect_cli = self.node.create_client(SetBool, '/yolo_world/execute')
        self.set_classes_cli = self.node.create_client(SetClasses, '/yolo_world/set_class')
        while (
            not self.execute_detect_cli.wait_for_service(timeout_sec=timeout_sec) or\
            not self.set_classes_cli.wait_for_service(timeout_sec=timeout_sec) 
        ):
            self.node.get_logger().error('Docker yolo_world_ros2 is not running.')
        
        # subscriber
        self.object_sub = self.node.create_subscription(ObjectPoseArray, '/yolo_world/object/poses', self._obj_cb, 10)
    
    def set_classes(self, classes: list) -> bool:
        try:
            self.set_classes_interface.classes = classes
            future = self.set_classes_cli.call_async(self.set_classes_interface)
            response: SetClasses.Response = None
            
            while rclpy.ok() and response is None:
                rclpy.spin_once(self.node)
                response = future.result()
            self.node.get_logger().info(f'Yolo detect classes: {classes}')
            return True
        except:
            self.node.get_logger().error(traceback.format_exc())
            return False
    
    def execute_yolo(self, exec: bool) -> bool:
        try:
            self.execute_interface.data = exec
            future = self.execute_detect_cli.call_async(self.execute_interface)
            response: SetBool.Response = None
            
            while rclpy.ok() and response is None:
                rclpy.spin_once(self.node)
                response = future.result()
            self.node.get_logger().info(response.message)
            return True
        except:
            self.node.get_logger().error(traceback.format_exc())
            return False
    
    def get_object_poses(self, classes: list, ref_frame:str='crane_x7_mounting_plate_link') -> list:
        try:
            self.set_classes(classes)
            self.execute_yolo(True)
            object_pose_list = []
            while rclpy.ok() and len(object_pose_list) == 0:
                rclpy.spin_once(self.node)
                if self._obj_poses is not None:
                    for obj_pose in self._obj_poses.poses:
                        print(obj_pose)
                        class_name = obj_pose.class_name
                        pred = obj_pose.predict
                        pose = obj_pose.pose

                        ref_pose = PoseStamped()
                        ref_pose.header = self._obj_poses.header
                        ref_pose.pose.position = pose
                        ref_pose.pose.orientation.w = 1.0 

                        try:
                            trans_pose = self.tf_buffer.transform(ref_pose, ref_frame)
                            object_pose_list.append((
                                trans_pose.pose.position.x,
                                trans_pose.pose.position.y,
                                trans_pose.pose.position.z,
                            ))
                        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                            self.node.get_logger().error(f'TF lookup failed: {e}')

            print(object_pose_list)
            return object_pose_list
        except:
            self.node.get_logger().error(traceback.format_exc())
            return None

    def _obj_cb(self, msg: ObjectPoseArray) ->None:
        self._obj_poses = msg

if __name__ == '__main__':
    from nakalab_crane_x7_common.arm_control import SimpleArmController

    rclpy.init()
    node = Node('sample_get_object_pose')
    arm = SimpleArmController()

    #arm.goal_state('home')

    get_obj = GetObjectPose(node)
    pose = get_obj.get_object_poses(['red_cube'])[0]
    get_obj.execute_yolo(False)

    arm.gripper(True)
    arm.ee_abs_pose(pose[0], pose[1], pose[2] + 0.15, 90.0, 0.0, 180.0)
    arm.ee_abs_pose(pose[0], pose[1], pose[2] + 0.08, 90.0, 0.0, 180.0)
    arm.gripper(False)
    arm.ee_abs_pose(pose[0], pose[1], pose[2]+ 0.15, 90.0, 0.0, 180.0)
    arm.gripper(True)
    arm.gripper(False)

    arm.goal_state('home')