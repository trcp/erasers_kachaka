#!/usr/bin/env python3

from kachaka_interfaces.action import ExecKachakaCommand
from kachaka_interfaces.msg import KachakaCommand
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped, Twist
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformListener, Buffer
from action_msgs.msg import GoalStatus
from rclpy_util.util import TemporarySubscriber
from rclpy.action import ActionClient
from rclpy.node import Node
import rclpy
import os
import math
import copy
import time
from typing import List, Tuple, Optional


NS = os.environ.get("KACHAKA_NAME")


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
        self.__action_client = ActionClient(self.__node, NavigateToPose, "/navigate_to_pose")
        self.__waypoints_client = ActionClient(self.__node, FollowWaypoints, "/follow_waypoints")
        self.__twist_publisher = self.__node.create_publisher(Twist, f'/{NS}/manual_control/cmd_vel', 10)

        if not self.__action_client.wait_for_server(timeout_sec=wait_time):
            self.__node.get_logger().fatal("Nav2 action server not available...")
            raise RuntimeError("Action server not available")
            
        try:
            if not self.__waypoints_client.wait_for_server(timeout_sec=5.0):
                self.__node.get_logger().warn("Follow waypoints action server not available...")
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
            future = self.__current_goal_handle.cancel_goal_async()
            try:
                rclpy.spin_until_future_complete(self.__node, future, timeout_sec=5.0)
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
        if self.__use_tf_for_pose:
            while rclpy.ok():
                rclpy.spin_once(self.__node, timeout_sec=0.1)
                try:
                    transform = self.__tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                    pose = PoseStamped()
                    pose.header = transform.header
                    pose.pose.position.x = transform.transform.translation.x
                    pose.pose.position.y = transform.transform.translation.y
                    pose.pose.position.z = transform.transform.translation.z
                    pose.pose.orientation = transform.transform.rotation
                    return pose
                except Exception as e:
                    continue
        else:
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

    def move_abs(self, x:float=0.0, y:float=0.0, yaw:float=0.0, wait:bool=True, consider_angle:bool=True) -> bool:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.__node.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        if consider_angle:
            q = quaternion_from_euler(0, 0, yaw)
        else:
            current_pose = self.get_current_pose()
            current_x = current_pose.pose.position.x
            current_y = current_pose.pose.position.y
            delta_x = x - current_x
            delta_y = y - current_y
            calculated_yaw = math.atan2(delta_y, delta_x) if (delta_x or delta_y) else 0.0
            q = quaternion_from_euler(0, 0, calculated_yaw)

        goal_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.__node.get_logger().info(
            f"ナビゲーション目標送出: X={x:.2f}, Y={y:.2f}, Yaw={math.degrees(yaw if consider_angle else calculated_yaw):.1f}°"
            + (" (角度考慮あり)" if consider_angle else " (角度考慮なし)")
        )

        goal_msg = NavigateToPose.Goal(pose=goal_pose)
        future = self.__action_client.send_goal_async(goal_msg)

        if wait:
            try:
                rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10.0)
                if not future.done():
                    self.__node.get_logger().error("Send goal timed out")
                    return False

                goal_handle = future.result()
                self.__current_goal_handle = goal_handle

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
                    self.__node.get_logger().info("Navigation succeeded, PID角度調整開始...")

                    # PID制御パラメータ
                    KP = 0.8    # 比例ゲイン
                    KI = 0.05   # 積分ゲイン
                    KD = 0.2    # 微分ゲイン
                    MAX_ANGULAR = 0.5  # 最大角速度[rad/s]
                    MIN_ANGULAR = 0.05 # 最小角速度[rad/s]
                    TOLERANCE = math.radians(1.0)  # 許容誤差[rad]
                    DT = 0.1  # 制御周期[s]

                    # 初期化
                    integral = 0.0
                    prev_error = 0.0
                    start_time = time.time()
                    last_time = start_time
                    max_adjust_time = 15.0

                    try:
                        while (time.time() - start_time) < max_adjust_time and rclpy.ok():
                            current_time = time.time()
                            dt = current_time - last_time
                            if dt < DT:
                                continue
                            
                            # 現在姿勢取得
                            current_pose = self.get_current_pose()
                            current_ori = current_pose.pose.orientation
                            current_q = [current_ori.x, current_ori.y, current_ori.z, current_ori.w]
                            _, _, current_yaw = euler_from_quaternion(current_q)

                            # 誤差計算
                            error = yaw - current_yaw
                            error = math.atan2(math.sin(error), math.cos(error))  # 正規化

                            # PID計算
                            P = KP * error
                            integral += KI * error * dt
                            derivative = KD * (error - prev_error) / dt

                            # 積分項の制限（アンチワインドアップ）
                            integral = max(min(integral, MAX_ANGULAR), -MAX_ANGULAR)

                            angular_z = P + integral + derivative

                            # 角速度制限
                            angular_z = max(min(angular_z, MAX_ANGULAR), -MAX_ANGULAR)
                            
                            # 最小速度以下で誤差が小さい場合は停止
                            if abs(error) < TOLERANCE:
                                angular_z = 0.0
                                break
                            elif abs(angular_z) < MIN_ANGULAR and abs(error) < math.radians(5):
                                angular_z = math.copysign(MIN_ANGULAR, angular_z)

                            # 速度指令発行
                            twist = Twist()
                            twist.angular.z = angular_z
                            self.__twist_publisher.publish(twist)

                            prev_error = error
                            last_time = current_time

                        else:
                            self.__node.get_logger().warn("角度調整タイムアウト")
                    finally:
                        # 最終停止処理
                        twist = Twist()
                        self.__twist_publisher.publish(twist)

                    return True
                else:
                    self.__node.get_logger().warn(f"ナビゲーション失敗 ステータスコード: {status}")
                    return False

            except KeyboardInterrupt:
                self.__node.get_logger().info("ナビゲーションをキャンセルします...")
                if self.cancel():
                    self.__node.get_logger().info("ナビゲーションキャンセル成功")
                else:
                    self.__node.get_logger().error("ナビゲーションキャンセル失敗")
                return False

            except Exception as e:
                self.__node.get_logger().error(f"ナビゲーションエラー: {str(e)}")
                return False
        else:
            future.add_done_callback(self.__goal_response_callback)
            try:
                rclpy.spin_until_future_complete(self.__node, future, timeout_sec=0.5)
            except Exception as e:
                self.__node.get_logger().debug(f"Spin interrupted: {str(e)}")
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
        return self.move_abs(new_x, new_y, new_yaw, wait)

    def create_waypoint(self, x:float, y:float, yaw:float) -> PoseStamped:
        waypoint = PoseStamped()
        waypoint.header.frame_id = "map"
        waypoint.header.stamp = self.__node.get_clock().now().to_msg()
        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, yaw)
        waypoint.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return waypoint
    
    def add_waypoint(self, waypoint:PoseStamped) -> None:
        self.__waypoints.append(waypoint)
        self.__node.get_logger().info(f"Waypoint added: ({waypoint.pose.position.x}, {waypoint.pose.position.y})")
    
    def add_waypoint_abs(self, x:float, y:float, yaw:float) -> None:
        waypoint = self.create_waypoint(x, y, yaw)
        self.add_waypoint(waypoint)
    
    def add_waypoint_rlt(self, x:float=0.0, y:float=0.0, yaw:float=0.0) -> None:
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
        self.add_waypoint_abs(new_x, new_y, new_yaw)
    
    def clear_waypoints(self) -> None:
        self.__waypoints = []
        self.__node.get_logger().info("All waypoints cleared")
    
    def get_waypoints(self) -> List[PoseStamped]:
        return self.__waypoints

    def execute_waypoints(self, reverse:bool=False, wait:bool=True) -> bool:
        if not self.__waypoints:
            self.__node.get_logger().warn("No waypoints to execute")
            return False

        processed_waypoints = [copy.deepcopy(wp) for wp in reversed(self.__waypoints)] if reverse else self.__waypoints
        if reverse:
            for wp in processed_waypoints:
                current_ori = wp.pose.orientation
                (_, _, yaw) = euler_from_quaternion([current_ori.x, current_ori.y, current_ori.z, current_ori.w])
                new_yaw = yaw + math.pi
                new_yaw = math.atan2(math.sin(new_yaw), math.cos(new_yaw))
                q = quaternion_from_euler(0, 0, new_yaw)
                wp.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        if self.__waypoints_client is None:
            try:
                self.__waypoints_client = ActionClient(self.__node, FollowWaypoints, "/follow_waypoints")
                if not self.__waypoints_client.wait_for_server(timeout_sec=3.0):
                    return False
            except Exception as e:
                return False
                
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = processed_waypoints
        future = self.__waypoints_client.send_goal_async(goal_msg)

        if wait:
            try:
                rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10.0)
                if not future.done():
                    return False
                goal_handle = future.result()
                self.__current_goal_handle = goal_handle
                if not goal_handle.accepted:
                    return False
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self.__node, result_future)
                return result.status == GoalStatus.STATUS_SUCCEEDED
            except Exception as e:
                return False
        else:
            future.add_done_callback(self.__goal_response_callback)
            rclpy.spin_once(self.__node, timeout_sec=0.1)
            return True
    
    def move_forward(self, speed:float, sec:float):
        twist = Twist()
        twist.linear.x = speed
        init_time = time.time()
        while rclpy.ok() and time.time() - init_time < sec:
            rclpy.spin_once(self.__node, timeout_sec=0.1)
            self.__twist_publisher.publish(twist)