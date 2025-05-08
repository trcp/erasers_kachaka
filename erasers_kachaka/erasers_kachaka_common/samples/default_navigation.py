#!/usr/bin/env python3

# ROS2 Python ライブラリをインポート
from rclpy.node import Node             # ノード制御
import rclpy                            # ROS2 制御クライアント.

# ナビゲーション API をインポート
from erasers_kachaka_common.navigator import DefaultNavigation


# rclpy を初期化
rclpy.init()

# ノードを宣言
# ノード名を "sample_kachaka_default_navigation" としている
node = Node("sample_kachaka_default_navigation")

# ナビゲーションを初期化
# ナビゲーションクラスの引数に node オブジェクトを渡さないとエラーになるので気をつけること
navigation = DefaultNavigation(node)

print(navigation.get_current_pose())

# ナビゲーションする
# 前に 50 cm 進む　
navigation.move_rlt(x=0.5)

print(navigation.get_current_pose())