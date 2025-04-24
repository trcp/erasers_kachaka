#!/usr/bin/env python3

# カチャカ制御用インターフェースをインポート
from kachaka_interfaces.action import ExecKachakaCommand
from kachaka_interfaces.msg import KachakaCommand

# nav2 アクションをインポート
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped

# TF 関連のライブラリをインポート
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformListener, Buffer

# Action
from action_msgs.msg import GoalStatus

# ROS2 Python ライブラリをインポート
from rclpy_util.util import TemporarySubscriber
from rclpy.action import ActionClient   # Action クライアント
from rclpy.node import Node             # ノード制御
import rclpy                            # ROS2 制御クライアント

import os                               # 環境変数取得用に os ライブライをインポート
import math
import copy
from typing import List, Tuple, Optional


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


class Nav2Navigation():
    def __init__(self, node:Node, exploration:bool=False, wait_time=10, tf_buffer:Buffer=None):
        self.__node = node
        self.__current_goal_handle = None
        self.__pose: PoseWithCovarianceStamped = None
        self.__waypoints: List[PoseStamped] = []
        self.__use_tf_for_pose = exploration 
        
        self.__tf_buffer = tf_buffer or Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self.__node)

        # 単一地点への移動用アクションクライアント
        self.__action_client = ActionClient(self.__node, NavigateToPose, "/navigate_to_pose")
        
        # ウェイポイント経由移動用アクションクライアント - 修正: 正確なトピック名を使用
        self.__waypoints_client = ActionClient(self.__node, FollowWaypoints, "/follow_waypoints")

        # NavigateToPose アクションサーバーが利用可能かチェック
        if not self.__action_client.wait_for_server(timeout_sec=wait_time):
            self.__node.get_logger().fatal("Nav2 action server not available...")
            raise RuntimeError("Action server not available")
            
        # FollowWaypoints アクションサーバーが利用可能かチェック (より長いタイムアウトを設定)
        try:
            if not self.__waypoints_client.wait_for_server(timeout_sec=5.0):
                self.__node.get_logger().warn("Follow waypoints action server not available...")
                self.__node.get_logger().warn("Waypoint functionality will not be available")
                # クライアントを None に設定してエラーが発生しても安全に動作するようにする
                self.__waypoints_client = None
            else:
                self.__node.get_logger().info("Successfully connected to follow_waypoints action server")
        except Exception as e:
            self.__node.get_logger().error(f"Error connecting to waypoints server: {str(e)}")
            self.__waypoints_client = None

    def __goal_response_callback(self, future):
        self.__current_goal_handle = future.result()
        if not self.__current_goal_handle.accepted:
            self.__node.get_logger().error("Goal rejected!")

    def cancel(self):
        if self.__current_goal_handle:
            self.__node.get_logger().info("Sending cancel request...")
            
            # キャンセルリクエストを送信
            future = self.__current_goal_handle.cancel_goal_async()
            
            try:
                # キャンセル結果を確実に待機
                rclpy.spin_until_future_complete(
                    self.__node, 
                    future,
                    timeout_sec=5.0
                )
                
                if future.done():
                    response = future.result()
                    
                    # キャンセル成功の確認
                    if response.goals_canceling:
                        self.__node.get_logger().info("Cancel request accepted")
                        self.__current_goal_handle = None  # ハンドルをリセット
                        return True
                    
                    self.__node.get_logger().warn(
                        f"Cancel failed with response: {response}"
                    )
                    return False
                
                self.__node.get_logger().error("Cancel request timed out")
                return False
                
            except Exception as e:
                self.__node.get_logger().error(f"Cancel failed: {str(e)}")
                return False
        return False

    def get_current_pose(self) -> PoseStamped:
        if self.__use_tf_for_pose:
            # TFを使用してbase_footprintの位置を取得
            while rclpy.ok():
                rclpy.spin_once(self.__node, timeout_sec=0.1)
                try:
                    transform = self.__tf_buffer.lookup_transform(
                        'map', 
                        'base_link', 
                        rclpy.time.Time()
                    )
                    pose = PoseStamped()
                    pose.header = transform.header
                    pose.pose.position.x = transform.transform.translation.x
                    pose.pose.position.y = transform.transform.translation.y
                    pose.pose.position.z = transform.transform.translation.z
                    pose.pose.orientation = transform.transform.rotation
                    return pose
                except Exception as e:
                    self.__node.get_logger().debug(f"TF lookup failed: {str(e)}")
                    continue
        else:
            # 従来の方法で位置を取得
            self.__pose = None

            def __cb(msg:PoseWithCovarianceStamped):
                self.__pose = msg
            
            with TemporarySubscriber(
                self.__node,
                msg=PoseWithCovarianceStamped,
                topic=f'/{NS}/pose',
                qos_profile=10,
                cb=__cb
            ):
                while rclpy.ok() and self.__pose is None:
                    rclpy.spin_once(self.__node, timeout_sec=0.1)
                
                pose = PoseStamped()
                pose.header = self.__pose.header
                pose.pose = self.__pose.pose.pose
            
            return pose

    def move_abs(self, x:float, y:float, yaw:float, wait:bool=True) -> bool:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.__node.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, yaw)
        goal_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        goal_msg = NavigateToPose.Goal(pose=goal_pose)
        future = self.__action_client.send_goal_async(goal_msg)

        if wait:
            try:
                rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10.0)
                if not future.done():
                    self.__node.get_logger().error("Send goal timed out")
                    return False

                goal_handle = future.result()
                self.__current_goal_handle = goal_handle  # 明示的にハンドルを更新

                if not goal_handle.accepted:
                    self.__node.get_logger().error("Goal rejected by server")
                    return False

                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self.__node, result_future)

                if not result_future.done():
                    self.__node.get_logger().error("Result timed out")
                    return False

                result = result_future.result()
                if result is None:
                    self.__node.get_logger().error("Action result is None")
                    return False

                status = result.status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    return True
                else:
                    self.__node.get_logger().warn(
                        f"Navigation failed with status: {status}")
                    return False

            except KeyboardInterrupt:
                self.__node.get_logger().info("Canceling navigation...")
                if self.cancel():
                    self.__node.get_logger().info("Navigation canceled")
                else:
                    self.__node.get_logger().error("Failed to cancel navigation")
                return False

            except Exception as e:
                self.__node.get_logger().error(f"Navigation error: {str(e)}")
                return False
        else:
            future.add_done_callback(self.__goal_response_callback)
            return True

    def move_rlt(self, x:float=0.0, y:float=0.0, yaw:float=0.0, wait:bool=True) -> bool:
        current_pose = self.get_current_pose()
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_orientation = current_pose.pose.orientation

        (_, _, current_yaw) = euler_from_quaternion([
            current_orientation.x,
            current_orientation.y,
            current_orientation.z,
            current_orientation.w
        ])

        new_x = current_x + x * math.cos(current_yaw) - y * math.sin(current_yaw)
        new_y = current_y + x * math.sin(current_yaw) + y * math.cos(current_yaw)
        new_yaw = current_yaw + yaw

        print(new_x, new_y, new_yaw)

        return self.move_abs(new_x, new_y, new_yaw, wait)
        
    def create_waypoint(self, x:float, y:float, yaw:float) -> PoseStamped:
        """
        新しいウェイポイントを作成します
        
        Args:
            x (float): X座標（メートル）
            y (float): Y座標（メートル）
            yaw (float): 方向（ラジアン）
            
        Returns:
            PoseStamped: 作成されたウェイポイント
        """
        waypoint = PoseStamped()
        waypoint.header.frame_id = "map"
        waypoint.header.stamp = self.__node.get_clock().now().to_msg()
        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.position.z = 0.0
        
        # ヨー角からクォータニオンに変換
        q = quaternion_from_euler(0, 0, yaw)
        waypoint.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        return waypoint
    
    def add_waypoint(self, waypoint:PoseStamped) -> None:
        """
        ウェイポイントをリストに追加します
        
        Args:
            waypoint (PoseStamped): 追加するウェイポイント
        """
        self.__waypoints.append(waypoint)
        self.__node.get_logger().info(f"Waypoint added: ({waypoint.pose.position.x}, {waypoint.pose.position.y})")
    
    def add_waypoint_abs(self, x:float, y:float, yaw:float) -> None:
        """
        絶対座標でウェイポイントを追加します
        
        Args:
            x (float): X座標（メートル）
            y (float): Y座標（メートル）
            yaw (float): 方向（ラジアン）
        """
        waypoint = self.create_waypoint(x, y, yaw)
        self.add_waypoint(waypoint)
    
    def add_waypoint_rlt(self, x:float=0.0, y:float=0.0, yaw:float=0.0) -> None:
        """
        現在位置からの相対座標でウェイポイントを追加します
        
        Args:
            x (float, optional): 前方向の距離（メートル）. Defaults to 0.0.
            y (float, optional): 左方向の距離（メートル）. Defaults to 0.0.
            yaw (float, optional): 回転角（ラジアン）. Defaults to 0.0.
        """
        current_pose = self.get_current_pose()
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_orientation = current_pose.pose.orientation

        (_, _, current_yaw) = euler_from_quaternion([
            current_orientation.x,
            current_orientation.y,
            current_orientation.z,
            current_orientation.w
        ])

        # 相対座標を絶対座標に変換
        new_x = current_x + x * math.cos(current_yaw) - y * math.sin(current_yaw)
        new_y = current_y + x * math.sin(current_yaw) + y * math.cos(current_yaw)
        new_yaw = current_yaw + yaw
        
        self.add_waypoint_abs(new_x, new_y, new_yaw)
    
    def clear_waypoints(self) -> None:
        """すべてのウェイポイントをクリアします"""
        self.__waypoints = []
        self.__node.get_logger().info("All waypoints cleared")
    
    def execute_waypoints(self, reverse:bool=False, wait:bool=True) -> bool:
        """
        登録済みのウェイポイントを順番に実行します
        
        Args:
            wait (bool, optional): 実行完了を待つかどうか. Defaults to True.
            reverse (bool, optional): 逆順で実行するかどうか. Defaults to False.
            
        Returns:
            bool: 成功したかどうか
        """
        if not self.__waypoints:
            self.__node.get_logger().warn("No waypoints to execute")
            return False
            
        # 逆順実行用の処理
        if reverse:
            # ディープコピーで元のリストを保護
            processed_waypoints = [copy.deepcopy(wp) for wp in reversed(self.__waypoints)]
            
            # 各ウェイポイントの向きを反転
            for wp in processed_waypoints:
                # 現在の姿勢をオイラー角に変換
                current_ori = wp.pose.orientation
                (_, _, yaw) = euler_from_quaternion(
                    [current_ori.x, current_ori.y, current_ori.z, current_ori.w]
                )
                
                # 方向を180度反転 (πラジアン加算)
                new_yaw = yaw + math.pi
                
                # 角度を正規化 (-π ~ π)
                new_yaw = math.atan2(math.sin(new_yaw), math.cos(new_yaw))
                
                # 新しいクォータニオンを設定
                q = quaternion_from_euler(0, 0, new_yaw)
                wp.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        else:
            processed_waypoints = self.__waypoints

        # アクションクライアントが正しく初期化されているか確認
        if self.__waypoints_client is None:
            self.__node.get_logger().error("Waypoint client not available")
            # 再接続を試みる
            try:
                self.__node.get_logger().info("Attempting to reconnect to follow_waypoints action server...")
                self.__waypoints_client = ActionClient(self.__node, FollowWaypoints, "/follow_waypoints")
                if not self.__waypoints_client.wait_for_server(timeout_sec=3.0):
                    self.__node.get_logger().error("Reconnection to follow_waypoints action server failed")
                    return False
                self.__node.get_logger().info("Successfully reconnected to follow_waypoints action server")
            except Exception as e:
                self.__node.get_logger().error(f"Reconnection error: {str(e)}")
                return False
            
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = processed_waypoints  # 処理済みウェイポイントを使用
        
        self.__node.get_logger().info(f"Executing {len(processed_waypoints)} waypoints {'in reverse' if reverse else ''}")
        future = self.__waypoints_client.send_goal_async(goal_msg)

        if wait:
            try:
                rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10.0)
                if not future.done():
                    self.__node.get_logger().error("Send waypoints goal timed out")
                    return False
                    
                goal_handle = future.result()
                self.__current_goal_handle = goal_handle
                
                if not goal_handle.accepted:
                    self.__node.get_logger().error("Waypoints goal rejected by server")
                    return False
                    
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self.__node, result_future)
                
                if not result_future.done():
                    self.__node.get_logger().error("Waypoints result timed out")
                    return False
                    
                result = result_future.result()
                if result is None:
                    self.__node.get_logger().error("Waypoints action result is None")
                    return False
                    
                # ウェイポイント実行の成功を確認
                if result.status == GoalStatus.STATUS_SUCCEEDED:
                    self.__node.get_logger().info("Waypoints navigation completed successfully")
                    return True
                else:
                    self.__node.get_logger().warn(
                        f"Waypoints navigation failed with status: {result.status}")
                    return False
                    
            except KeyboardInterrupt:
                self.__node.get_logger().info("Canceling waypoints navigation...")
                if self.cancel():
                    self.__node.get_logger().info("Waypoints navigation canceled")
                else:
                    self.__node.get_logger().error("Failed to cancel waypoints navigation")
                return False
                
            except Exception as e:
                self.__node.get_logger().error(f"Waypoints navigation error: {str(e)}")
                return False
        else:
            future.add_done_callback(self.__goal_response_callback)
            return True
    
    def get_waypoints(self) -> List[PoseStamped]:
        """
        現在のウェイポイントリストを取得します
        
        Returns:
            List[PoseStamped]: ウェイポイントのリスト
        """
        return self.__waypoints