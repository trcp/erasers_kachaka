#!/usr/bin/env python3

# rclpy
from rclpy.node import Node
import rclpy

# tf
import tf2_ros

# module
from erasers_kachaka_common.vision.object_detect import ObjectDetect
from erasers_kachaka_common.tts import TTS
import smach

# general
import time
import traceback

class ObjectDetectState(smach.State):
    def __init__(self,
                 node:Node=Node,
                 tf_buffer:tf2_ros.Buffer=None,
                 timeout_sec:float=10.0,
                 ref_frame:str='crane_x7_mounting_plate_link',
                 say_fn: TTS.say=None,
                 ) -> None:
        smach.State.__init__(self,
                             outcomes=["success", "timeout", "failure"],
                             input_keys=["serch_object_dict"],
                             output_keys=["object_poses"])
        self.node = node
        self.ref_frame = ref_frame
        self.say_fn = say_fn
        tf_buffer = tf2_ros.Buffer() if tf_buffer is None else tf_buffer
        self.object_detect = ObjectDetect(node, tf_buffer, timeout_sec)
    
    def execute(self, ud):
        try:
            serch_object_dict:dict = ud.serch_object_dict

            if self.say_fn is not None: self.say_fn("オブジェクトを探しています")

            classes = list(serch_object_dict.keys())
            predicts = list(serch_object_dict.values())
            object_poses = self.object_detect.get_object_infomation(
                classes, min(predicts), self.ref_frame
            )

            if self.say_fn is not None: self.say_fn("オブジェクトを見つけました")

            ud.object_poses = object_poses 
            return 'success'
        except:
            self.node.get_logger().error(traceback.format_exc())
            return 'failure'

if __name__ == "__main__":
    from nakalab_crane_x7_common.arm_control import SimpleArmController
    from erasers_kachaka_common.state.wait_push_hand import WaitPushHand
    from erasers_kachaka_common.tts import TTS

    rclpy.init()
    node = Node("sample_object_detect_state")

    sm = smach.StateMachine(outcomes=['success', 'failure'])

    # シミュレーションかどうか
    sm.userdata.is_sim = False

    # 検出したい物体リスト
    sm.userdata.serch_object_dict = {'red_cube':0.5}

    controller = SimpleArmController()
    tts = TTS()
    say = tts.say

    with sm:
        smach.StateMachine.add('WAIT_PUSH_HAND', WaitPushHand(node, controller, 120, 0.3, say, "プッシュ", "物体検出を開始します"),
                               transitions={'push':'SERCHOBJECTS',
                                            'timeout':'failure',
                                            'failure':'failure'})

        smach.StateMachine.add('SERCHOBJECTS', ObjectDetectState(node, say_fn=say),
                               transitions={'success':'success',
                                            'timeout':'failure',
                                            'failure':'failure'})
     
    outcomes = sm.execute()
    print(outcomes)