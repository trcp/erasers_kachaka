#!/usr/bin/env python3
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy_util.util import TemporarySubscriber, TemporaryApproximateTimeSynchronizer
from rclpy.node import Node
import rclpy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_srvs.srv import SetBool

from cv_bridge import CvBridge

import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
import os

from typing import Union, List, Tuple, Type


NS = os.environ.get("KACHAKA_NAME") 


class RobotStopper():
    """
    Kachaka を前後に微速移動させ、そのばでロボットを固定させます。
    ロボットと正常に接続できなかった場合 RuntimeError がスローされます。

    Args:
        node: rclpy.node.Node : rclpy Node オブジェクト。代入必須です。
    """
    def __init__(self, node:Node):
        self.__node = node

        self.__client = self.__node.create_client(SetBool, f'/{NS}/robot_stopper')
        while not self.__client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().error('May be KACHAKA is not running ...')
            raise RuntimeError('May be KACHAKA is not running ...')

    def __send_req(self, req:SetBool.Request):
        future = self.__client.call_async(req)

        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10.0)
        if future.done():
            res = future.result()
            self.__node.get_logger().info(res.message)
            return True
        else:
            return False
    
    
    def enable(self)->bool:
        """_summary_
        カチャカをその場で固定させます。

        Returns:
            bool: ストッパーが動作すると True。なにか問題が発生すると False をスローします。
        """
        req = SetBool.Request()
        req.data = True

        return self.__send_req(req)


    def disable(self)->bool:
        """
        カチャカの固定を解除します。

        Returns:
            bool: ストッパーが停止すると True。なにか問題が発生すると False をスローします。
        """
        req = SetBool.Request()
        req.data = False

        return self.__send_req(req)


class RobotInfo():
    """
    Kachaka の情報を取得します。

    Args:
        node: rclpy.node.Node : rclpy Node オブジェクト。代入必須です。
    """
    def __init__(self, node:Node):
        self.__node = node
        self.__ns = NS
    

    def battery(self)->float:
        """_summary_
        Kachaka のバッテリー残量を取得します。

        Returns:
            float: kachaka のバッテリー残量 [%]
        """
        timeout = 3.0
        self.__battery = None
        def __cb(msg:Float32):
            self.__battery = msg.data
        
        with TemporarySubscriber(self.__node,
                                 Float32,
                                 f'/{self.__ns}/robot_info/battery',
                                 10,
                                 __cb):
            init_time = time.time()
            while self.__battery is None and time.time() - init_time < timeout:
                rclpy.spin_once(self.__node, timeout_sec=0.1)
        
        return self.__battery


class Camera():
    """
    Kachaka のカメラ情報を取得します。

    Args:
        node: rclpy.node.Node : rclpy Node オブジェクト。代入必須です。
    """
    def __init__(self, node:Node):
        self.__node = node
        self.__ns = NS
        self.__bridge = CvBridge()
        self.__qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
    

    def get_camera(self, front_camera:bool=True, back_camera:bool=True, tof_camera:bool=False, use_msg:bool=False, preview:bool=False)->List[Union[Image, np.ndarray]]:
        """
        Kachaka からのカメラ画像を取得します。指定された引数に応じて同一時刻における画像を取得します。

        Args:
            front_camera (bool, optional): 前方カメラ情報を取得する場合は True. Defaults to True.
            back_camera (bool, optional): 後方カメラ情報を取得する場合は True. Defaults to True.
            tof_camera (bool, optional): 前方ToFカメラ情報を取得する場合は True. Defaults to False.
            use_msg (bool, optional): 返り値を Image メッセージとして出力する場合は True. Defaults to False.
            preview (bool, optional): プレビューを有効にする場合は True. Defaults to False.

        Returns:
            List[Union[Image, np.ndarray]]: カメラ画像情報 Image or ndarray
        """
        timeout = 3.0
        self.__front_camera_image_msg = None
        self.__front_camera_image = None
        self.__back_camera_image_msg = None
        self.__back_camera_image = None
        self.__tof_camera_image_msg = None
        self.__tof_camera_image = None

        topic_list = []

        if not any([front_camera, back_camera, tof_camera]):
            self.__node.get_logger().warn('All camera args False')
            return []
        
        if front_camera: topic_list.append((Image, f'/{self.__ns}/front_camera/image_raw'))
        if back_camera: topic_list.append((Image, f'/{self.__ns}/back_camera/image_raw'))
        if tof_camera: topic_list.append((Image, f'/{self.__ns}/tof_camera/image_raw'))

        data_received = False

        def __cb(*msgs):
            nonlocal data_received

            for (msg_type, topic_name), msg in zip(topic_list, msgs):
                if 'front_camera' in topic_name:
                    self.__front_camera_image_msg = msg
                    self.__front_camera_image = self.__bridge.imgmsg_to_cv2(msg, 'bgr8')
                elif 'back_camera' in topic_name:
                    self.__back_camera_image_msg = msg
                    self.__back_camera_image = self.__bridge.imgmsg_to_cv2(msg, 'bgr8')
                elif 'tof_camera' in topic_name:
                    self.__tof_camera_image_msg = msg
                    self.__tof_camera_image = self.__bridge.imgmsg_to_cv2(msg, 'bgr8')

            data_received = True

        with TemporaryApproximateTimeSynchronizer(self.__node,
                                                  topic_list,
                                                  self.__qos,
                                                  0.1,
                                                  __cb):
            init_time = time.time()
            while not data_received and time.time() - init_time < timeout:
                rclpy.spin_once(self.__node, timeout_sec=0.1)
        
        if not data_received:
            self.__node.get_logger().error('Failure get camera data')
            return []
        
        image_list = [self.__front_camera_image,
                      self.__back_camera_image,
                      self.__tof_camera_image]
        if preview:
            valid_images = [img for img in image_list if img is not None]
            num_images = len(valid_images)

            if num_images > 0:
                fig, axes = plt.subplots(1, num_images, figsize=(5 * num_images, 5))
                fig.suptitle("Camera's Preview")

                if num_images == 1:
                    axes = [axes]
                
                for ax, image in zip(axes, valid_images):
                    ax.imshow(image) # BGRからRGBに変換
                    ax.axis('off') # 軸を非表示に

                plt.show()
        
        if use_msg: image_list = [self.__front_camera_image_msg,
                      self.__back_camera_image_msg,
                      self.__tof_camera_image_msg]
            
        return image_list



    def get_front_camera(self, use_msg:bool=False, preview:bool=False)->Union[Image, np.ndarray]:
        """
        前方のカメラ画像を取得します。

        Args:
            use_msg (bool, optional): 返り値を Image メッセージとして出力する場合は True. Defaults to False.
            preview (bool, optional): プレビューを有効にする場合は True. Defaults to False.

        Returns:
            Union[Image, np.ndarray]: カメラ画像情報 Image or ndarray
        """
        timeout = 3.0
        self.__front_camera_image_msg = None
        self.__front_camera_image = None
        def __cb(msg:Float32):
            self.__front_camera_image_msg = msg
            
        with TemporarySubscriber(self.__node,
                                 Image,
                                 f'/{self.__ns}/front_camera/image_raw',
                                 self.__qos,
                                 __cb):
            init_time = time.time()
            while self.__front_camera_image_msg is None and time.time() - init_time < timeout:
                rclpy.spin_once(self.__node, timeout_sec=0.1)
        if self.__front_camera_image_msg is None:
            self.__node.get_logger().error('Failure get front camera')
            return self.__front_camera_image
        
        self.__front_camera_image = self.__bridge.imgmsg_to_cv2(self.__front_camera_image_msg, 'bgr8')
        if preview:
            plt.figure("Front Camera Preview") 
            plt.imshow(self.__front_camera_image)
            plt.axis('off')
            plt.show()
        if use_msg: self.__front_camera_image = self.__front_camera_image_msg       

        return self.__front_camera_image
    

    def get_back_camera(self, use_msg:bool=False, preview:bool=False)->Union[Image, np.ndarray]:
        """
        後方のカメラ画像を取得します。

        Args:
            use_msg (bool, optional): 返り値を Image メッセージとして出力する場合は True. Defaults to False.
            preview (bool, optional): プレビューを有効にする場合は True. Defaults to False.

        Returns:
            Union[Image, np.ndarray]: カメラ画像情報 Image or ndarray
        """
        timeout = 3.0
        self.__back_camera_image_msg = None
        self.__back_camera_image = None
        def __cb(msg:Float32):
            self.__back_camera_image_msg = msg
            
        with TemporarySubscriber(self.__node,
                                 Image,
                                 f'/{self.__ns}/back_camera/image_raw',
                                 self.__qos,
                                 __cb):
            init_time = time.time()
            while self.__back_camera_image_msg is None and time.time() - init_time < timeout:
                rclpy.spin_once(self.__node, timeout_sec=0.1)
        
        if self.__back_camera_image_msg is None:
            self.__node.get_logger().error('Failure get back camera')
            return self.__back_camera_image
        
        self.__back_camera_image = self.__bridge.imgmsg_to_cv2(self.__back_camera_image_msg, 'bgr8')
        if preview:
            plt.figure("back Camera Preview") 
            plt.imshow(self.__back_camera_image)
            plt.axis('off')
            plt.show()
        if use_msg: self.__back_camera_image = self.__back_camera_image_msg       

        return self.__back_camera_image


    def get_tof_camera(self, use_msg:bool=False, preview:bool=False)->Union[Image, np.ndarray]:
        """
        前方 ToF のカメラ画像を取得します。

        Args:
            use_msg (bool, optional): 返り値を Image メッセージとして出力する場合は True. Defaults to False.
            preview (bool, optional): プレビューを有効にする場合は True. Defaults to False.

        Returns:
            Union[Image, np.ndarray]: カメラ画像情報 Image or ndarray
        """
        timeout = 3.0
        self.__tof_camera_image_msg = None
        self.__tof_camera_image = None
        def __cb(msg:Float32):
            self.__tof_camera_image_msg = msg
            
        with TemporarySubscriber(self.__node,
                                 Image,
                                 f'/{self.__ns}/tof_camera/image_raw',
                                 self.__qos,
                                 __cb):
            init_time = time.time()
            while self.__tof_camera_image_msg is None and time.time() - init_time < timeout:
                rclpy.spin_once(self.__node, timeout_sec=0.1)
        
        if self.__tof_camera_image_msg is None:
            self.__node.get_logger().error('Failure get tof camera')
            return self.__tof_camera_image
        
        self.__tof_camera_image = self.__bridge.imgmsg_to_cv2(self.__tof_camera_image_msg, 'bgr8')
        if preview:
            plt.figure("tof Camera Preview") 
            plt.imshow(self.__tof_camera_image)
            plt.axis('off')
            plt.show()
        if use_msg: self.__tof_camera_image = self.__tof_camera_image_msg       

        return self.__tof_camera_image
