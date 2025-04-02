#!/usr/bin/env python
# -*- coding: utf-8 -*-
from rclpy.node import Node
import rclpy

from std_msgs.msg import String
import time

miss_count = 0
def recog_cb(msg):
    global miss_count
    text = msg.data
    print(f"recived:{text}\n")
    if "Lifting" in text or "lifting" in text:
        print("LIFT")
    elif "one" in text or "1" in text :
        print("1")
    elif "two" in text or "2" in text :
        print("22")
    elif miss_count == 5:
        print("MISS")
    else:
        miss_count +=1

def main():
    rclpy.init()
    node = Node("recog_node")
    sub = node.create_subscription(String, "/voice_recog", recog_cb, 10)
    try :
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
