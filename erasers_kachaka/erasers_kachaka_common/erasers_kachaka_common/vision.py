#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Pose, TransformStamped

import smach
import tf2_ros
from tf2_ros import TransformListener, Buffer
from transforms3d.quaternions import quat2mat
from tf_transformations import euler_from_quaternion

import time
import math

# for reference
from typing import Optional

# for reference
from erasers_kachaka_common.navigation import SimpleNavigator
from erasers_kachaka_common.tts import TTS


class WaitArucoTopics(smach.State):
    def __init__(self, node: Node, timeout: float=10.0, direction: str="front", say_fn: TTS.say=None) -> None:
        smach.State.__init__(self, outcomes=["success", "timeout"],
                            output_keys=["aruco_topic", "ref_camera_frame"]
                             )

        self.node = node
        self.timeout = timeout
        self.direction = direction
        self.say = say_fn

    def execute(self, userdata) -> str:
        userdata.aruco_topic = None
        find_topic = False
        say = self.say
        if say is not None: say("トピックを確認しています")

        init_time = time.time()
        while rclpy.ok() and not find_topic and time.time() - init_time <= self.timeout:
            topic_list = self.node.get_topic_names_and_types()
            for topic in topic_list:
                if self.direction in topic[0] and "ros2_aruco_interfaces/msg/ArucoMarkers" in topic[1][0]:
                    if self.direction == "front":
                        userdata.ref_camera_frame = "camera_front_link"
                    elif self.direction == "back":
                        userdata.ref_camera_frame = "camera_back_link"
                    else:
                        self.node.get_logger().warn(f"There were no frames that could be referenced from the camera position “{self.direction}” specified in the 'direction' argument.")
                        userdata.ref_camera_frame = None

                    find_topic = True
                    userdata.aruco_topic = topic[0]
        
        if not find_topic:
            if say is not None: say("タイムアウトです。Aruco ノードが動作していません")
            return "timeout"
        
        return "success"

class ApproachArucoMarker(smach.State):
    def __init__(self, node: Node,
                        id: int=0,
                        timeout: float=10.0,
                        attempts: int=3, 
                        navigator: SimpleNavigator=None,
                        tf_buff: Buffer=None,
                        say_fn=None
                ) -> None:
        smach.State.__init__(self, outcomes=["success", "failure", "notfound"],
                             input_keys=["aruco_topic", "ref_camera_frame"]
                             )

        self.node = node
        self.target_id = id
        self.timeout = timeout
        self.attempts = attempts
        self.navigator = navigator
        self.tf_buffer = tf_buff
        self.say = say_fn
        self.pose: Optional[Pose] = None

        # add tf_buff
        if self.tf_buffer is None:
            self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def _cb_maerkers(self, msg: ArucoMarkers) -> None:
        for id, pose in zip(msg.marker_ids, msg.poses):
            if id == self.target_id:
                self.pose = pose

    def execute(self, userdata) -> str:
        self.node.get_logger().info("Subscribe marker topic: " + userdata.aruco_topic)
        self.node.get_logger().info("referenced camera frame: " + userdata.ref_camera_frame)
        marker_sub = self.node.create_subscription(ArucoMarkers, userdata.aruco_topic, self._cb_maerkers, 10)
        say = self.say
        arrived_flag = False

        # check include navigaor
        if self.navigator is None:
            self.node.get_logger().error("Not included SimpleNavigator API")
            return "failure"

        # serch marker
        for i in range(self.attempts):
            if self.pose is not None:
                break

            if say is not None: say("マーカーを探しています", True)

            self.node.get_logger().info("Serch marker ...")
            init_time = time.time()
            while time.time() - init_time <= self.timeout:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if self.pose is not None:
                    self.node.get_logger().info("Marker is found !")
                    if say is not None: say("マーカーを見つけました！", True)
                    break
            i += 1
        
        # approach to marker via navigator
        if not self.pose is None:
            # クォータニオンから回転行列に変換
            q = (self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)
            p = (self.pose.position.x, self.pose.position.y, self.pose.position.z)
            from_reference_orientation = quat2mat(q)
            from_reference_position = from_reference_orientation.dot(p)

            transform: TransformStamped = None
            while rclpy.ok() and transform is None:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        "base_footprint",
                        userdata.ref_camera_frame,
                        rclpy.time.Time()
                    )
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException,  tf2_ros.ExtrapolationException) as e:
                    self.node.get_logger().error(str(e))
                    continue
                    
                bq = (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w)
                bp = (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)

                base_rotate = quat2mat(bq)
                target_pose = base_rotate.dot(from_reference_position) + bp

                rotation = euler_from_quaternion(q)
                print(math.degrees(rotation[2]+1.57))
                
                self.node.get_logger().info(f"""
Target marker Pose fron map
position:
    x: {target_pose[0]}
    y: {target_pose[1]}
    z: {target_pose[2]}
                """)

        self.node.destroy_subscription(marker_sub)

        if self.pose is not None:
            if say is not None: say("マーカーへ移動します。")
            arrived_flag = self.navigator.go_rlt(
                target_pose[0]-0.35,
                target_pose[1],
                rotation[2]+1.57
            )
        
        if self.pose is None:
            if say is not None: say("マーカーが見つかりませんでした。")
            return "notfound"
        if not arrived_flag:
            if say is not None: say("マーカーにアプローチできませんでした。")
            return "failure"
        return "success"


def demo():
    from erasers_kachaka_common.tts import TTS
    from erasers_kachaka_common.navigation import SimpleNavigator
    
    rclpy.init()
    node = Node("common_vision_demo")

    sm = smach.StateMachine(outcomes=["success", "failure"])

    # activate spaeker
    tts = TTS()
    say = tts.say

    # activate navigator
    navigator = SimpleNavigator()

    with sm:
        smach.StateMachine.add("WAIT", WaitArucoTopics(node, timeout=10.0, say_fn=say),
                                transitions={
                                    "success":"APPROACH",
                                    "timeout":"failure"
                                },
                                remapping={
                                    "aruco_topic":"aruco_topic",
                                    "ref_camera_frame":"ref_camera_frame"
                                }
        )
        smach.StateMachine.add("APPROACH", ApproachArucoMarker(node, id=2, navigator=navigator, say_fn=say),
                                transitions={
                                    "success":"success",
                                    "failure":"failure",
                                    "notfound":"failure"
                                },
                                remapping={
                                    "aruco_topic":"aruco_topic",
                                    "ref_camera_frame":"ref_camera_frame"
                                }
        )

    outcome = sm.execute()
    if outcome == "success":
        say("タスクを終了します", False)
    else:
        say("すみません。タスクに失敗しました。", False)

    node.destroy_node()
if __name__ == "__main__":
    demo()
