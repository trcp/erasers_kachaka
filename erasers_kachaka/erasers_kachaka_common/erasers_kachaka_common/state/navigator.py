#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from erasers_kachaka_common.navigator import Nav2Navigation, DefaultNavigation
from erasers_kachaka_common.tts import TTS

import smach


class Nav2MoveAbsState(smach.State):
    def __init__(self, 
                    node:Node,
                    navigation:Nav2Navigation,
                    tts_say:TTS.say,
                    start_msg:str = '目標地点へ移動します。',
                    success_msg:str = '目標地点に到達しました。',
                    failure_msg:str = '目標地点へ到達できませんでした。'
                 ):
        
        smach.State.__init__(
            self, outcomes=['success', 'failure'],
            input_keys=['abs_pose']
        )

        self._node = node
        self._navigation = navigation
        self._say = tts_say
        self._start_msg = start_msg
        self._success_msg = success_msg
        self._failure_msg = failure_msg
    

    def execute(self, userdata):

        self._say(self._start_msg)

        x = userdata.abs_pose[0]
        y = userdata.abs_pose[1]
        yaw = userdata.abs_pose[2]
        result = self._navigation.move_abs(x, y, yaw)

        if result:
            self._say(self._success_msg)
            return 'success'

        else:
            self._say(self._failure_msg)
            return 'failure'


class Nav2MoveRltState(smach.State):
    def __init__(self, 
                    node:Node,
                    navigation:Nav2Navigation,
                    tts_say:TTS.say,
                    start_msg:str = '目標地点へ移動します。',
                    success_msg:str = '目標地点に到達しました。',
                    failure_msg:str = '目標地点へ到達できませんでした。'
                 ):
        
        smach.State.__init__(
            self, outcomes=['success', 'failure'],
            input_keys=['abs_pose']
        )

        self._node = node
        self._navigation = navigation
        self._say = tts_say
        self._start_msg = start_msg
        self._success_msg = success_msg
        self._failure_msg = failure_msg
    

    def execute(self, userdata):

        self._say(self._start_msg)

        x = userdata.abs_pose[0]
        y = userdata.abs_pose[1]
        yaw = userdata.abs_pose[2]
        result = self._navigation.move_rlt(x, y, yaw)

        if result:
            self._say(self._success_msg)
            return 'success'

        else:
            self._say(self._failure_msg)
            return 'failure'