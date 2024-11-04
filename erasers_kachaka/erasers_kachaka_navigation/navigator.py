#!/usr/bin/env python3

# message
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

# TF
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformListener, Buffer, TransformException

# rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
import rclpy

# general
import math
import time
import signal

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('ngsr_simple_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 現在のゴールのハンドルを保存するための変数
        self.current_goal_handle = None

        # SIGINT シグナル (Ctrl+C) をキャッチして停止メソッドを呼び出す
        signal.signal(signal.SIGINT, self.stop_navigation)
        #self.destroy_node()

    def __send_action(self, pose):
        """
        位置情報を送信
        """
        try:
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose

            self._action_client.wait_for_server()

            send_goal_future = self._action_client.send_goal_async(goal_msg)
            
            rclpy.spin_until_future_complete(self, send_goal_future)
            
            self.current_goal_handle = send_goal_future.result()
            if not self.current_goal_handle.accepted:
                self.get_logger().info('Goal rejected :(')
                return
            self.get_logger().info('Goal accepted :)')

            get_result_future = self.current_goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)
            result = get_result_future.result().result

            time.sleep(1)

            # ナビゲーションの成功・失敗を判断
            if self.current_goal_handle.status == 4:  # 成功の場合のステータスコード（通常は4）
                self.get_logger().info('Reached Goal')
                return True
            else:
                self.get_logger().error('Failure ... status code : %d'%self.current_goal_handle.status)
                return False
        except Exception:
            self.get_logger().error('Failure. some error is occured')
            return False

    def stop_navigation(self, signum=None, frame=None):
        """
        ナビゲーションを停止
        """
        if self.current_goal_handle:
            self.get_logger().warn('Cancelling current goal...')
            cancel_future = self.current_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            self.get_logger().warn('Goal cancelled')
        else:
            self.get_logger().warn('No active goal to cancel')

        if signum and frame:
            self.destroy_node()

    def get_current_pose(self, xyy=False):
        while rclpy.ok():
            rclpy.spin_once(self)
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                if xyy:
                    p = []
                    p.append(transform.transform.translation.x)
                    p.append(transform.transform.translation.y)

                    e = euler_from_quaternion((transform.transform.rotation.x,
                                                transform.transform.rotation.y,
                                                transform.transform.rotation.z,
                                                transform.transform.rotation.w))
                    p.append(e[2])

                    transform = p
                    
                return transform
            except Exception as e:
                pass


    def go_abs(self, x=0.0, y=0.0, yaw=0.0, degrees=False):
        """
        絶対座標でロボットの目標地点を指定
        """
        # degrees が True なら yaw を弧度法に変換
        if degrees:
            yaw = math.radians(yaw)
        # 四元数に変換
        q = quaternion_from_euler(0.0,0.0,yaw)
        # 目標座標を設定
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]

        return self.__send_action(p)

    def go_rlt(self, x=0, y=0, yaw=0, degrees=False):
        """
        相対座標でロボットの目標地点を指定
        """
        # degrees が True なら yaw を弧度法に変換
        if degrees:
            yaw = math.radians(yaw)
        p = self.get_current_pose()

        px = p.transform.translation.x
        py = p.transform.translation.y

        e = euler_from_quaternion((p.transform.rotation.x,
                                   p.transform.rotation.y,
                                   p.transform.rotation.z,
                                   p.transform.rotation.w))
        pyaw = e[2]

        # 相対座標を回転を考慮して計算
        x_new = px + (x * math.cos(pyaw) - y * math.sin(pyaw))
        y_new = py + (x * math.sin(pyaw) + y * math.cos(pyaw))
        yaw_new = pyaw + yaw

        q = quaternion_from_euler(0.0, 0.0, yaw_new)

        # 目標座標を設定
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x_new
        pose.pose.position.y = y_new
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return self.__send_action(pose)

#class DockingNavigator(Node):

if __name__ == "__main__":
    rclpy.init()
    sn = SimpleNavigator()
    sn.go_abs(y=-4.0, degrees=True)
    sn.go_abs(degrees=True)
