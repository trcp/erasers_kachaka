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

# ナビゲーションする（相対）
# 前に 50 cm 進む
print("move forward")
navigation.move_rlt(x=0.5)

# 後方に 50 cm 進む　
print("move back")
navigation.move_rlt(x=-0.5)

# 左に 50 cm 進む　
print("move left")
navigation.move_rlt(y=0.5)

# 右に 50 cm 進む　
print("move right")
navigation.move_rlt(y=-0.5)

# 90度左旋回　
print("move right")
navigation.move_rlt(yaw=1.57)

# 90度右旋回　
print("move right")
navigation.move_rlt(yaw=-1.57)

# ナビゲーションする（絶対座標）
# すべての引数を 0.0 にすると充電ドッグに乗り上げてしまう。
print("move base of front")
navigation.move_abs(0.1, 0.0, 0.0)

print("move forward")
navigation.move_abs(0.5, 0.0, 0.0)

print(navigation.get_current_pose())