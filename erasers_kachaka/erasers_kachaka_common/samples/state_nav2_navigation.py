#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from erasers_kachaka_common.state.navigator import Nav2MoveAbsState, Nav2MoveRltState
from erasers_kachaka_common.navigator import Nav2Navigation, DefaultNavigation
from erasers_kachaka_common.tts import TTS

import smach


def main():
    rclpy.init()
    node = Node('sample_state_nav2_navigation')

    tts = TTS(node)
    say = tts.say
    navigation = Nav2Navigation(node, exploration=False)

    sm = smach.StateMachine(outcomes=['success', 'failure'])

    sm.userdata.abs_pose = [0.0, 0.0, 1.57]


    with sm:

        smach.StateMachine.add('MOVE_ABS', Nav2MoveAbsState(node=node,
                                                            navigation=navigation,
                                                            tts_say=say),
                                    transitions={
                                        'success': 'MOVE_RLT',
                                        'failure': 'failure'
                                    }
                                )

        smach.StateMachine.add('MOVE_RLT', Nav2MoveRltState(node=node,
                                                            navigation=navigation,
                                                            tts_say=say),
                                    transitions={
                                        'success': 'success',
                                        'failure': 'failure'
                                    }
                                )
    
    outcome = sm.execute()
    if outcome == 'success':
        say('タスク成功')
    
    else:
        say('タスクに失敗しました')
    
    node.destroy_node()


if __name__ == '__main__':
    main()