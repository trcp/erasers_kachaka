#!/usr/bin/nv python3
import rclpy.logging
from rclpy.node import Node
import rclpy

from sensor_msgs.msg import JointState

import smach

from nakalab_crane_x7_common.arm_control import SimpleArmController
from erasers_kachaka_common.tts import TTS

import time
import traceback

class WaitPushHand(smach.State):
    def __init__(self,
                 node: Node=None,
                 controller: SimpleArmController=None,
                 timeout_sec=120,
                 threthhold: float=0.4,
                 say_fn: TTS.say=None,
                 prompt_msg: str="Please push my hand.",
                 success_msg: str="Thank you.") -> None:
        smach.State.__init__(self, outcomes=['push', 'timeout', 'failure'],
                            input_keys=['is_sim'],
                            output_keys=[])
        
        self.node = node
        self.controller = SimpleArmController() if controller is None else controller
        self.say_fn = say_fn
        self.prompt_msg = prompt_msg
        self.success_msg = success_msg
        self.timeout_sec = timeout_sec
        self.threthhold = threthhold
        self.force = None
    
    def _joint_status_cb(self, msg: JointState) ->None:
        for joint_name, force in zip(msg.name, msg.effort):
            if joint_name == 'crane_x7_shoulder_revolute_part_tilt_joint':
                self.force = force
    
    def execute(self, ud):
        # rclpy.node.Node オブジェクトが代入されているか確認
        if self.node is None:
            rclpy.logging.get_logger('wait_push_hand').error('node オブジェクトを代入してください')
            return 'failure'
        # say_fn ビルド
        if self.say_fn is None:
            tts = TTS()
            self.say_fn = tts.say

        try:
            trans_push_hand = self.controller.goal_state('push_hand', 2.0)
            if not trans_push_hand:
                self.node.get_logger().info('Failure translate push hand pose ...')
                return "failure"
            
            if self.prompt_msg:
                self.say_fn(self.prompt_msg)

            joint_status_sub = self.node.create_subscription(JointState, 'joint_states', self._joint_status_cb, 10)

            # シミュレーションかどうか確認
            if ud.is_sim:
                self.node.get_logger().warn('THIS IS SIMULATION')
                joint_status_sub.destroy()
                return 'push'
            else:
                self.node.get_logger().info('Wait Push hanbd ...')
                init_time = time.time()
                while rclpy.ok():
                    rclpy.spin_once(self.node, timeout_sec=0.1)

                    if self.force is not None:
                        #print(self.force)
                        if self.force > self.threthhold:
                            self.node.get_logger().info('Push !')
                            break

                    if time.time() - init_time > self.timeout_sec:
                        self.node.get_logger().warn('Tiumeout ...')
                        joint_status_sub.destroy()
                        return 'timeout'
                    
                joint_status_sub.destroy()
                if self.success_msg:
                    self.say_fn(self.success_msg, False)
                return 'push'
        except:
            rclpy.logging.get_logger('wait_push_hand').error(traceback.format_exc())
            return 'failure'

if __name__ == '__main__':
    rclpy.init()
    node = Node('sample_state')

    sm = smach.StateMachine(outcomes=['success', 'failure'])

    # シミュレーションかどうか
    sm.userdata.is_sim = False

    controller = SimpleArmController()

    with sm:
        smach.StateMachine.add('WAIT_PUSH_HAND', WaitPushHand(node, controller, 120, 0.3, None, "僕の手をプッシュしてください", "おしてくれてありがとう！"),
                               transitions={'push':'success',
                                            'timeout':'failure',
                                            'failure':'failure'})
     
    outcomes = sm.execute()
    controller.goal_state('home', 1.0)

    print(outcomes)