#!/usr/bin/env python3

# ROS2 Python ライブラリをインポート
from rclpy.node import Node             # ノード制御
import rclpy                            # ROS2 制御クライアント.

# ナビゲーション API をインポート
from erasers_kachaka_common.navigator import Nav2Navigation


# rclpy を初期化
rclpy.init()

# ノードを宣言
# ノード名を "sample_kachaka_default_navigation" としている
node = Node("sample_kachaka_nav2_navigation")

# ナビゲーションを初期化
# ナビゲーションクラスの引数に node オブジェクトを渡さないとエラーになるので気をつけること
navigation = Nav2Navigation(node)

# ナビゲーションする
result = navigation.move_abs(0.0, 0.0, 0.0) # マップ原点へ移動
print(result) # 結果が True なら成功

result = navigation.move_abs(0.0, 0.0, 1.57) # マップ原点で90度左方向へ旋回
print(result)

result = navigation.move_rlt(yaw=-1.57, wait=True) # 現在の姿勢から90度右費方向へ旋回
print(result)