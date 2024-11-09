#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

from erasers_kachaka_common.tts import TTS

def speak_sample():
    rclpy.init()

    tts = TTS()
    say = tts.say

    say("こんちゃ")
