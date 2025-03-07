#!/usr/bin/env python3

# カチャカ制御用インターフェースをインポート
from kachaka_interfaces.action import ExecKachakaCommand
from kachaka_interfaces.msg import KachakaCommand

# ROS2 Python ライブラリをインポート
from rclpy.action import ActionClient   # Action クライアント
from rclpy.node import Node             # ノード制御
import rclpy                            # ROS2 制御クライアント

import os                               # 環境変数取得用に os ライブライをインポート


NS = os.environ.get("KACHAKA_NAME")     # 名前空間の取得


# カチャカ内蔵ナビゲーションを使用するクラス
class DefaultNavigation():
    def __init__(self, node:Node, wait_time=10):
        self.__node = node                                              # 引数に渡された Node オブジェクトを使用

        # Action クライアントの作成
        self.__action_client = ActionClient(self.__node, ExecKachakaCommand, "/%s/kachaka_command/execute"%NS)

        # もし wait_time の時間分 Action サーバーからの応答がない場合カチャカと正常に動作していないことを警告する
        if not self.__action_client.wait_for_server(timeout_sec=wait_time):
            self.__node.get_logger().fatal("May be KACHAKA is not running ...")
        
        # アクションアーバーに渡すコマンドタイプを定義
        self.__cmd = KachakaCommand()
        self.__cmd.command_type = KachakaCommand.MOVE_TO_POSE_COMMAND   # ナビゲーション用コマンドを指定

        self.__goal_msg = ExecKachakaCommand.Goal()                     # サーバーに渡すメッセージインターフェースを定義
    

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
