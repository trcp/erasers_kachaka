#!/usr/bin/env python3

# ================================================
# ROS2 基本モジュール
# ================================================
import rclpy  # ROS2 Pythonクライアントライブラリ
from rclpy.node import Node  # ROS2ノードの基本クラス
from rclpy.action import ActionClient  # アクションクライアントの実装
from action_msgs.msg import GoalStatus  # アクションゴールの状態定義

# ================================================
# カチャカ固有インターフェース
# ================================================
from kachaka_interfaces.action import ExecKachakaCommand  # カチャカコマンド実行アクション
from kachaka_interfaces.msg import KachakaCommand  # カチャカコマンドメッセージ

# ================================================
# ナビゲーション関連
# ================================================
from nav2_msgs.action import NavigateToPose, FollowWaypoints  # NAV2ナビゲーションアクション
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped  # 位置姿勢情報
from geometry_msgs.msg import Twist  # 速度指令

# ================================================
# 座標変換関連
# ================================================
from geometry_msgs.msg import TransformStamped  # 座標変換情報
from tf2_ros import TransformListener, Buffer  # TF2変換リスナーとバッファ
from tf_transformations import euler_from_quaternion, quaternion_from_euler  # クォータニオン変換ユーティリティ

# ================================================
# ユーティリティモジュール
# ================================================
from rclpy_util.util import TemporarySubscriber  # 一時的なサブスクライバー
import os  # OS依存機能
import math  # 数学関数
import copy  # オブジェクトコピー
import time  # 時間関連関数
from typing import List, Tuple, Optional  # 型ヒント


NS = os.environ.get("KACHAKA_NAME")


# カチャカ内蔵ナビゲーションを使用するクラス
class DefaultNavigation():
    def __init__(self, node:Node, wait_time=10, tf_buffer:Buffer=None):
        """DefaultNavigation クラスのコンストラクタ

        Args:
            node (Node): ROS2ノードオブジェクト
            wait_time (int, optional): アクションサーバー接続待機時間(秒). Defaults to 10.
            tf_buffer (Buffer, optional): TF2バッファオブジェクト. Noneの場合は新規作成. Defaults to None.

        Raises:
            RuntimeError: アクションサーバーに接続できない場合
        """
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
    

    def get_current_pose(self) -> TransformStamped:
        """現在のロボットの位置姿勢を取得

        Returns:
            TransformStamped: map座標系から見たbase_footprintのTF変換情報
            
        Note:
            このメソッドはTFツリーが利用可能になるまでブロッキングします
            定期的にrclpy.spin_once()を呼び出してコールバックを処理します
        """
        while rclpy.ok():
            rclpy.spin_once(self.__node, timeout_sec=0.1)

            try:
                transform = self.__tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                return transform
            except Exception as e:
                self.__node.get_logger().debug(str(e))
                pass
    

    def move_abs(self, x:float, y:float, yaw:float, wait:bool=True) -> bool:
        """指定された絶対座標に移動

        Args:
            x (float): 目標位置のX座標(m)
            y (float): 目標位置のY座標(m)
            yaw (float): 目標姿勢のヨー角(rad)
            wait (bool, optional): 移動完了まで待機するか. Defaults to True.

        Returns:
            bool: 移動が成功した場合はTrue、失敗した場合はFalse
            
        Raises:
            RuntimeError: ゴールがサーバーに拒否された場合
        """
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


    def move_rlt(self, x:float=0.0, y:float=0.0, yaw:float=0.0, wait:bool=True) -> bool:
        """現在位置からの相対座標で移動

        Args:
            x (float, optional): 前方向への移動量(m). Defaults to 0.0.
            y (float, optional): 左方向への移動量(m). Defaults to 0.0.
            yaw (float, optional): 反時計回りの回転量(rad). Defaults to 0.0.
            wait (bool, optional): 移動完了まで待機するか. Defaults to True.

        Returns:
            bool: 移動が成功した場合はTrue、失敗した場合はFalse
            
        Note:
            移動量はロボットの現在の姿勢を基準に計算されます
            例えば、ロボットが90度向いている状態でx=1.0を指定すると、
            ロボットの左方向に1m移動します
        """
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
        return self.move_abs(new_x, new_y, new_yaw, wait)


class Nav2Navigation():
    """Nav2を使用したロボットナビゲーションを管理するクラス
    
    Nav2のNavigateToPoseアクションとFollowWaypointsアクションを使用して、
    絶対座標/相対座標での移動、ウェイポイント登録と実行、手動制御などの機能を提供します。
    TF2を使用した現在位置の取得や、ナビゲーション後の微調整PID制御も実装しています。
    
    Attributes:
        __node (Node): ROS2ノードオブジェクト
        __current_goal_handle (ClientGoalHandle): 現在実行中のゴールハンドル
        __pose (PoseWithCovarianceStamped): 現在の姿勢情報
        __waypoints (List[PoseStamped]): 登録されたウェイポイントリスト
        __use_tf_for_pose (bool): TFを使用して姿勢を取得するかどうか
        __tf_buffer (Buffer): TF2バッファ
        __tf_listener (TransformListener): TF2リスナー
        __action_client (ActionClient): NavigateToPoseアクションクライアント
        __waypoints_client (ActionClient): FollowWaypointsアクションクライアント
        __twist_publisher (Publisher): 手動制御用のTwistパブリッシャー
    """
    
    
    def __init__(self, node:Node, exploration:bool=False, wait_time=10, tf_buffer:Buffer=None):
        """Nav2Navigationクラスのコンストラクタ

        Args:
            node (Node): ROS2ノードオブジェクト
            exploration (bool, optional): 探索モードかどうか。Trueの場合TFを使用して姿勢を取得。
                                        Defaults to False.
            wait_time (int, optional): アクションサーバー接続待機時間(秒). Defaults to 10.
            tf_buffer (Buffer, optional): TF2バッファオブジェクト. Noneの場合は新規作成.
                                        Defaults to None.

        Raises:
            RuntimeError: NavigateToPoseアクションサーバーに接続できない場合
        """
        # ノードインスタンスをプライベート変数に保存
        self.__node = node
        
        # 現在のゴールハンドルと姿勢情報を初期化
        self.__current_goal_handle = None
        self.__pose: PoseWithCovarianceStamped = None
        
        # ウェイポイントリストを初期化
        self.__waypoints: List[PoseStamped] = []
        
        # 姿勢取得方法の設定（TF使用フラグ）
        self.use_tf_for_pose = exploration 
        
        # TFバッファの初期化（引数で指定がない場合は新規作成）
        self.__tf_buffer = tf_buffer or Buffer()
        
        # TFリスナーの初期化
        self.__tf_listener = TransformListener(self.__tf_buffer, self.__node)
        
        # NavigateToPoseアクションクライアントの作成
        self.__action_client = ActionClient(self.__node, NavigateToPose, "/navigate_to_pose")
        
        # FollowWaypointsアクションクライアントの作成
        self.__waypoints_client = ActionClient(self.__node, FollowWaypoints, "/follow_waypoints")
        
        # 手動制御用のTwistパブリッシャーの作成
        self.__twist_publisher = self.__node.create_publisher(Twist, f'/{NS}/manual_control/cmd_vel', 10)

        # ナビゲーションアクションサーバーの接続確認
        if not self.__action_client.wait_for_server(timeout_sec=wait_time):
            self.__node.get_logger().fatal("Nav2 action server not available...")
            raise RuntimeError("Action server not available")
            
        try:
            # ウェイポイントアクションサーバーの接続確認
            if not self.__waypoints_client.wait_for_server(timeout_sec=5.0):
                self.__node.get_logger().warn("Follow waypoints action server not available...")
                self.__waypoints_client = None
        except Exception as e:
            self.__node.get_logger().error(f"Error connecting to waypoints server: {str(e)}")
            self.__waypoints_client = None

    
    def __goal_response_callback(self, future):
        """アクションゴールのレスポンスコールバック

        Args:
            future (Future): ゴールハンドルのFutureオブジェクト
        """
        # ゴールハンドルを取得して保存
        self.__current_goal_handle = future.result()
        
        # ゴールが拒否された場合のエラーログ出力
        if not self.__current_goal_handle.accepted:
            self.__node.get_logger().error("Goal rejected!")

    
    def cancel(self) -> bool:
        """現在実行中のナビゲーションをキャンセル

        Returns:
            bool: キャンセルが成功した場合はTrue、失敗した場合はFalse
        """
        # 現在のゴールハンドルが存在する場合のみ処理
        if self.__current_goal_handle:
            # 非同期でゴールキャンセルを実行
            future = self.__current_goal_handle.cancel_goal_async()
            try:
                # キャンセル結果を待機
                rclpy.spin_until_future_complete(self.__node, future, timeout_sec=5.0)
                
                # キャンセル結果を確認
                if future.done():
                    response = future.result()
                    if response.goals_canceling:
                        self.__current_goal_handle = None
                        return True
                    return False
                return False
            except Exception as e:
                self.__node.get_logger().error(f"Cancel failed: {str(e)}")
                return False
        return False

    
    def get_current_pose(self) -> PoseStamped:
        """現在のロボットの位置姿勢を取得

        Returns:
            PoseStamped: map座標系における現在の姿勢
            
        Note:
            explorationモードがTrueの場合はTFから姿勢を取得し、
            Falseの場合は/amcl_poseトピックから姿勢を取得します。
            取得できない場合はブロッキング状態になります。
        """
        # TFを使用して姿勢を取得する場合
        if self.use_tf_for_pose:
            while rclpy.ok():
                # ROSコールバックを処理
                rclpy.spin_once(self.__node, timeout_sec=0.1)
                try:
                    # mapからbase_linkへのTF変換を取得
                    transform = self.__tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                    
                    # TF変換結果をPoseStampedに変換
                    pose = PoseStamped()
                    pose.header = transform.header
                    pose.pose.position.x = transform.transform.translation.x
                    pose.pose.position.y = transform.transform.translation.y
                    pose.pose.position.z = transform.transform.translation.z
                    pose.pose.orientation = transform.transform.rotation
                    return pose
                except Exception as e:
                    continue
        # トピックから姿勢を取得する場合
        else:
            self.__pose = None
            
            # 一時的なサブスクライバーのコールバック関数
            def __cb(msg:PoseWithCovarianceStamped):
                self.__pose = msg
                
            # 一時的なサブスクライバーを作成
            with TemporarySubscriber(
                self.__node,
                msg=PoseWithCovarianceStamped,
                topic=f'/{NS}/pose',
                qos_profile=10,
                cb=__cb
            ):
                # 姿勢データが取得できるまで待機
                while rclpy.ok() and self.__pose is None:
                    rclpy.spin_once(self.__node, timeout_sec=0.1)
                    
                # 取得した姿勢をPoseStampedに変換
                pose = PoseStamped()
                pose.header = self.__pose.header
                pose.pose = self.__pose.pose.pose
            return pose

    
    def move_abs(self, x:float=0.0, y:float=0.0, yaw:float=0.0, wait:bool=True, consider_angle:bool=True) -> bool:
        """指定された絶対座標に移動

        Args:
            x (float, optional): 目標位置のX座標(m). Defaults to 0.0.
            y (float, optional): 目標位置のY座標(m). Defaults to 0.0.
            yaw (float, optional): 目標姿勢のヨー角(rad). Defaults to 0.0.
            wait (bool, optional): 移動完了まで待機するか. Defaults to True.
            consider_angle (bool, optional): 目標角度を考慮するか. Falseの場合、目標位置への方向を自動計算.
                                          Defaults to True.

        Returns:
            bool: 移動が成功した場合はTrue、失敗した場合はFalse
            
        Note:
            consider_angleがFalseの場合、目標位置への方向を自動計算して姿勢を決定します。
            ナビゲーション完了後、PID制御を使用して角度の微調整を行います。
        """
        # 目標姿勢メッセージの作成
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.__node.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        # 角度考慮の有無でクォータニオン計算方法を変更
        if consider_angle:
            # 指定角度でクォータニオンを計算
            q = quaternion_from_euler(0, 0, yaw)
        else:
            # 現在位置から目標位置への方向を計算
            current_pose = self.get_current_pose()
            current_x = current_pose.pose.position.x
            current_y = current_pose.pose.position.y
            delta_x = x - current_x
            delta_y = y - current_y
            calculated_yaw = math.atan2(delta_y, delta_x) if (delta_x or delta_y) else 0.0
            q = quaternion_from_euler(0, 0, calculated_yaw)

        # 姿勢メッセージにクォータニオンを設定
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        # アクションゴールメッセージを作成
        goal_msg = NavigateToPose.Goal(pose=goal_pose)
        
        # 非同期でゴールを送信
        future = self.__action_client.send_goal_async(goal_msg)

        # 結果を待機する場合
        if wait:
            try:
                # ゴール送信結果を待機
                rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10.0)
                if not future.done():
                    self.__node.get_logger().error("Send goal timed out")
                    return False

                # ゴールハンドルを取得
                goal_handle = future.result()
                self.__current_goal_handle = goal_handle

                # ゴールが拒否された場合
                if not goal_handle.accepted:
                    self.__node.get_logger().error("Goal rejected by server")
                    return False

                # 実行結果を待機
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self.__node, result_future)

                # 結果が返ってきていない場合
                if not result_future.done():
                    self.__node.get_logger().error("Result timed out")
                    return False

                # 結果を取得
                result = result_future.result()
                if result is None:
                    self.__node.get_logger().error("Action result is None")
                    return False

                # ステータスを確認
                status = result.status
                if status == GoalStatus.STATUS_SUCCEEDED:

                    # PID制御パラメータ設定
                    KP = 0.8    # 比例ゲイン
                    KI = 0.05   # 積分ゲイン
                    KD = 0.2    # 微分ゲイン
                    MAX_ANGULAR = 0.5  # 最大角速度[rad/s]
                    MIN_ANGULAR = 0.05 # 最小角速度[rad/s]
                    TOLERANCE = math.radians(1.0)  # 許容誤差[rad]
                    DT = 0.1  # 制御周期[s]

                    # PID制御変数の初期化
                    integral = 0.0
                    prev_error = 0.0
                    start_time = time.time()
                    last_time = start_time
                    max_adjust_time = 15.0

                    try:
                        # 角度調整ループ
                        while (time.time() - start_time) < max_adjust_time and rclpy.ok():
                            current_time = time.time()
                            dt = current_time - last_time
                            if dt < DT:
                                continue
                            
                            # 現在姿勢を取得
                            current_pose = self.get_current_pose()
                            current_ori = current_pose.pose.orientation
                            current_q = [current_ori.x, current_ori.y, current_ori.z, current_ori.w]
                            _, _, current_yaw = euler_from_quaternion(current_q)

                            # 角度誤差を計算（正規化）
                            error = yaw - current_yaw
                            error = math.atan2(math.sin(error), math.cos(error))

                            # PID計算
                            P = KP * error
                            integral += KI * error * dt
                            derivative = KD * (error - prev_error) / dt

                            # 積分項の制限（アンチワインドアップ）
                            integral = max(min(integral, MAX_ANGULAR), -MAX_ANGULAR)

                            # 角速度を計算
                            angular_z = P + integral + derivative

                            # 角速度を制限
                            angular_z = max(min(angular_z, MAX_ANGULAR), -MAX_ANGULAR)
                            
                            # 許容誤差以下の場合停止
                            if abs(error) < TOLERANCE:
                                angular_z = 0.0
                                break
                            # 最小速度以下の場合でもある程度の誤差があれば最小速度を維持
                            elif abs(angular_z) < MIN_ANGULAR and abs(error) < math.radians(5):
                                angular_z = math.copysign(MIN_ANGULAR, angular_z)

                            # 速度指令を発行
                            twist = Twist()
                            twist.angular.z = angular_z
                            self.__twist_publisher.publish(twist)

                            # 前回誤差を更新
                            prev_error = error
                            last_time = current_time

                        else:
                            self.__node.get_logger().warn("角度調整タイムアウト")
                    finally:
                        # 最終的に停止指令を発行
                        twist = Twist()
                        self.__twist_publisher.publish(twist)

                    return True
                else:
                    self.__node.get_logger().warn(f"ナビゲーション失敗 ステータスコード: {status}")
                    return False

            except KeyboardInterrupt:
                return False

            except Exception as e:
                self.__node.get_logger().error(f"ナビゲーションエラー: {str(e)}")
                return False
        else:
            # 非同期モードの場合、コールバックを登録
            future.add_done_callback(self.__goal_response_callback)
            try:
                rclpy.spin_until_future_complete(self.__node, future, timeout_sec=0.5)
            except Exception as e:
                self.__node.get_logger().debug(f"Spin interrupted: {str(e)}")
            return True

    
    def move_rlt(self, x:float=0.0, y:float=0.0, yaw:float=0.0, wait:bool=True) -> bool:
        """現在位置からの相対座標で移動

        Args:
            x (float, optional): 前方向への移動量(m). Defaults to 0.0.
            y (float, optional): 左方向への移動量(m). Defaults to 0.0.
            yaw (float, optional): 反時計回りの回転量(rad). Defaults to 0.0.
            wait (bool, optional): 移動完了まで待機するか. Defaults to True.

        Returns:
            bool: 移動が成功した場合はTrue、失敗した場合はFalse
            
        Note:
            移動量はロボットの現在の姿勢を基準に計算されます。
            内部的にはmove_abs()を呼び出して移動を実行します。
        """
        # 現在姿勢を取得
        current_pose = self.get_current_pose()
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_orientation = current_pose.pose.orientation
        
        # 現在のヨー角をクォータニオンから計算
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
        
        # 絶対座標移動を実行
        return self.move_abs(new_x, new_y, new_yaw, wait)

    
    def create_waypoint(self, x:float, y:float, yaw:float) -> PoseStamped:
        """ウェイポイントを作成

        Args:
            x (float): ウェイポイントのX座標(m)
            y (float): ウェイポイントのY座標(m)
            yaw (float): ウェイポイントのヨー角(rad)

        Returns:
            PoseStamped: 作成されたウェイポイント姿勢
        """
        # ウェイポイントメッセージを作成
        waypoint = PoseStamped()
        waypoint.header.frame_id = "map"
        waypoint.header.stamp = self.__node.get_clock().now().to_msg()
        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.position.z = 0.0

        # クォータニオンを計算して設定
        q = quaternion_from_euler(0, 0, yaw)
        waypoint.pose.orientation.x = q[0]
        waypoint.pose.orientation.y = q[1]
        waypoint.pose.orientation.z = q[2]
        waypoint.pose.orientation.w = q[3]
        return waypoint
    
    
    def add_waypoint(self, waypoint:PoseStamped) -> None:
        """ウェイポイントをリストに追加

        Args:
            waypoint (PoseStamped): 追加するウェイポイント姿勢
        """
        # ウェイポイントリストに追加
        self.__waypoints.append(waypoint)
        
        # ログに追加情報を出力
        self.__node.get_logger().info(f"Waypoint added: ({waypoint.pose.position.x}, {waypoint.pose.position.y})")
    
    
    def add_waypoint_abs(self, x:float, y:float, yaw:float) -> None:
        """絶対座標でウェイポイントを追加

        Args:
            x (float): ウェイポイントのX座標(m)
            y (float): ウェイポイントのY座標(m)
            yaw (float): ウェイポイントのヨー角(rad)
        """
        # ウェイポイントを作成して追加
        waypoint = self.create_waypoint(x, y, yaw)
        self.add_waypoint(waypoint)
    
    
    def add_waypoint_rlt(self, x:float=0.0, y:float=0.0, yaw:float=0.0) -> None:
        """現在位置からの相対座標でウェイポイントを追加

        Args:
            x (float, optional): 前方向への移動量(m). Defaults to 0.0.
            y (float, optional): 左方向への移動量(m). Defaults to 0.0.
            yaw (float, optional): 反時計回りの回転量(rad). Defaults to 0.0.
        """
        # 現在姿勢を取得
        current_pose = self.get_current_pose()
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_orientation = current_pose.pose.orientation
        
        # 現在のヨー角を計算
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
        
        # 絶対座標でウェイポイントを追加
        self.add_waypoint_abs(new_x, new_y, new_yaw)
    
    
    def clear_waypoints(self) -> None:
        """登録されている全てのウェイポイントをクリア"""
        # ウェイポイントリストをクリア
        self.__waypoints = []
        
        # ログにクリア情報を出力
        self.__node.get_logger().info("All waypoints cleared")
    

    def get_waypoints(self) -> List[PoseStamped]:
        """登録されているウェイポイントリストを取得

        Returns:
            List[PoseStamped]: ウェイポイント姿勢のリスト
        """
        # ウェイポイントリストを返す
        return self.__waypoints



    def execute_waypoints(self, reverse:bool=False, wait:bool=True) -> bool:
        """登録されたウェイポイントを順番に実行

        Args:
            reverse (bool, optional): 逆順で実行するか. Defaults to False.
            wait (bool, optional): 実行完了まで待機するか. Defaults to True.

        Returns:
            bool: 実行が成功した場合はTrue、失敗した場合はFalse
            
        Note:
            reverseがTrueの場合、ウェイポイントの順序と姿勢が反転されます。
            ウェイポイントサーバーが利用できない場合、自動的に再接続を試みます。
        """
        # ウェイポイントが空の場合
        if not self.__waypoints:
            self.__node.get_logger().warn("No waypoints to execute")
            return False

        # 逆順実行の場合、ウェイポイントを反転してコピー
        processed_waypoints = [copy.deepcopy(wp) for wp in reversed(self.__waypoints)] if reverse else self.__waypoints
        
        # 逆順実行の場合、各ウェイポイントの姿勢を反転
        if reverse:
            for waypoint in processed_waypoints:
                current_ori = waypoint.pose.orientation
                (_, _, yaw) = euler_from_quaternion([current_ori.x, current_ori.y, current_ori.z, current_ori.w])
                
                # ヨー角を180度反転
                new_yaw = yaw + math.pi
                new_yaw = math.atan2(math.sin(new_yaw), math.cos(new_yaw))
                
                # 新しいクォータニオンを計算
                q = quaternion_from_euler(0, 0, new_yaw)
                waypoint.pose.orientation.x = q[0]
                waypoint.pose.orientation.y = q[1]
                waypoint.pose.orientation.z = q[2]
                waypoint.pose.orientation.w = q[3]
                
        # ウェイポイントクライアントが存在しない場合、再接続を試みる
        if self.__waypoints_client is None:
            try:
                self.__waypoints_client = ActionClient(self.__node, FollowWaypoints, "/follow_waypoints")
                if not self.__waypoints_client.wait_for_server(timeout_sec=3.0):
                    return False
            except Exception as e:
                return False
                
        # ゴールメッセージを作成
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = processed_waypoints
        
        # 非同期でゴールを送信
        future = self.__waypoints_client.send_goal_async(goal_msg)

        # 結果を待機する場合
        if wait:
            try:
                # ゴール送信を待機
                rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10.0)
                if not future.done():
                    return False
                    
                # ゴールハンドルを取得
                goal_handle = future.result()
                self.__current_goal_handle = goal_handle
                
                # ゴールが拒否された場合
                if not goal_handle.accepted:
                    return False
                
                # 結果を待機
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self.__node, result_future)
                
                # 結果が返ってきていない場合
                if not result_future.done():
                    return False
                    
                # 結果を取得
                result = result_future.result()
                if result is None:
                    return False
                    
                # 成功ステータスを返す
                return result.status == GoalStatus.STATUS_SUCCEEDED
            except Exception as e:
                self.__node.get_logger().error(f"Waypoints execution failed: {str(e)}")
                return False
        else:
            # 非同期モードの場合、コールバックを登録
            future.add_done_callback(self.__goal_response_callback)
            rclpy.spin_once(self.__node, timeout_sec=0.1)
            return True
    

    def move_forward(self, speed:float, sec:float):
        """指定した速度で指定時間前進

        Args:
            speed (float): 移動速度(m/s)
            sec (float): 移動時間(秒)
        """
        # 速度指令メッセージを作成
        twist = Twist()
        twist.linear.x = speed
        
        # 開始時間を記録
        init_time = time.time()
        
        # 指定時間まで速度指令を発行
        while rclpy.ok() and time.time() - init_time < sec:
            rclpy.spin_once(self.__node, timeout_sec=0.1)
            self.__twist_publisher.publish(twist)
