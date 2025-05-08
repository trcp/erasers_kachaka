#!/usr/bin/env python3

# ROS2 Python ライブラリをインポート
from rclpy.node import Node             # ノード制御
import rclpy                            # ROS2 制御クライアント.

# ナビゲーション API をインポート
from erasers_kachaka_common.navigator import Nav2Navigation

import time


# rclpy を初期化
rclpy.init()

# ノードを宣言
# ノード名を "sample_kachaka_default_navigation" としている
node = Node("sample_kachaka_nav2_navigation")

# ナビゲーションを初期化
# ナビゲーションクラスの引数に node オブジェクトを渡さないとエラーになるので気をつけること
navigation = Nav2Navigation(node, exploration=True)

# Waypoint 作成
init_time = time.time()
while time.time() - init_time < 15:
    navigation.add_waypoint_rlt()
    time.sleep(3)


navigation.execute_waypoints(reverse=True)

navigation.move_abs(0.0,0.0,0.0)