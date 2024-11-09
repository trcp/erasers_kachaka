#!/usr/bin/env python3

# rclpy と Node をインポート
from rclpy.node import Node
import rclpy

# Navigation API をインポート
from erasers_kachaka_common.navigation import SimpleNavigator

def simple_navigation_sample():
    # rclpy を初期化してノードを作成
    rclpy.init()
    node = Node("sample_navigation")

    # Navigator を初期化
    sn = SimpleNavigator()

    # 現在位置から前へ進む
    sn.go_rlt(x=1.5)

    sn.stop_navigation()