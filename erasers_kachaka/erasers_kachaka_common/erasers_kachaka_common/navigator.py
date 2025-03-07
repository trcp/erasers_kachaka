#!/usr/bin/env python3

# カチャカ制御用インターフェースをインポート
from kachaka_interfaces.action import ExecKachakaCommand
from kachaka_interfaces.msg import KachakaCommand

# TF 関連のライブラリをインポート
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformListener, Buffer

# ROS2 Python ライブラリをインポート
from rclpy.action import ActionClient   # Action クライアント
from rclpy.node import Node             # ノード制御
import rclpy                            # ROS2 制御クライアント

import os                               # 環境変数取得用に os ライブライをインポート
import math


NS = os.environ.get("KACHAKA_NAME")     # 名前空間の取得


# カチャカ内蔵ナビゲーションを使用するクラス
class DefaultNavigation():
    def __init__(self, node:Node, wait_time=10, tf_buffer:Buffer=None):
        self.__node = node                                              # 引数に渡された Node オブジェクトを使用
        
        # TF2 セットアップ
        self.__tf_buffer = tf_buffer
        if self.__tf_buffer is None:
            self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self.__node)

        # Action クライアントの作成
        self.__action_client = ActionClient(self.__node, ExecKachakaCommand, "/%s/kachaka_command/execute"%NS)

        # もし wait_time の時間分 Action サーバーからの応答がない場合カチャカと正常に動作していないことを警告する
        if not self.__action_client.wait_for_server(timeout_sec=wait_time):
            self.__node.get_logger().fatal("May be KACHAKA is not running ...")
        
        # アクションアーバーに渡すコマンドタイプを定義
        self.__cmd = KachakaCommand()
        self.__cmd.command_type = KachakaCommand.MOVE_TO_POSE_COMMAND   # ナビゲーション用コマンドを指定

        self.__goal_msg = ExecKachakaCommand.Goal()                     # サーバーに渡すメッセージインターフェースを定義
    

    # 現在のロボットの座標を取得
    def get_current_pose(self) -> TransformStamped:
        while rclpy.ok():
            rclpy.spin_once(self.__node, timeout_sec=0.1)

            try:
                transform = self.__tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                return transform
            except Exception as e:
                self.__node.get_logger().debug(str(e))
                pass
    

    # 絶対座標で移動
    def move_abs(self, x:float, y:float, yaw:float, wait:bool=True) -> bool:
        # 引数で与えられた座標をコマンドに代入
        self.__cmd.move_to_pose_command_x = x
        self.__cmd.move_to_pose_command_y = y
        self.__cmd.move_to_pose_command_yaw = yaw

        # 座標コマンドをメッセージに代入する
        self.__goal_msg.kachaka_command = self.__cmd

        future = self.__action_client.send_goal_async(self.__goal_msg)  # メッセージをアクションサーバーに送信

        # wait 引数が True なら結果がくるまで待機する
        if wait:
            rclpy.spin_until_future_complete(self.__node, future)
            goal_handle = future.result()

            # サーバーから拒否されたらエラーを出す
            if not goal_handle.accepted:
                self.__node.get_logger().error("Goal rejected by server")
                return False

            # サーバーからの実行結果を待機
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.__node, result_future)
            result = result_future.result().result

            # 実行結果を返す
            if result.success:
                return True
            else:
                self.__node.get_logger().warn("Navigation failed")
                return False


    # 相対座標で移動
    def move_rlt(self, x:float=0.0, y:float=0.0, yaw:float=0.0, wait:bool=True) -> bool:

        # 現在の座標を取得
        current_pose: TransformStamped = self.get_current_pose()

        # 現在の位置と姿勢を取得
        current_x = current_pose.transform.translation.x
        current_y = current_pose.transform.translation.y
        current_orientation = current_pose.transform.rotation

        # クォータニオンをオイラー角に変換
        (_, _, current_yaw) = euler_from_quaternion([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])

        # 相対座標を絶対座標に変換
        # 新しい位置を計算
        new_x = current_x + x * math.cos(current_yaw) - y * math.sin(current_yaw)
        new_y = current_y + x * math.sin(current_yaw) + y * math.cos(current_yaw)
        new_yaw = current_yaw + yaw

        # 移動
        self.move_abs(new_x, new_y, new_yaw, wait)