#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


NAMESPACE = os.environ.get('KACHAKA_NAME')


slam_params = {
    'erk_slam_toolbox': {
        'ros__parameters': {
            # プラグイン関連設定
            'solver_plugin': 'solver_plugins::CeresSolver',
            'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
            'ceres_preconditioner': 'SCHUR_JACOBI',
            'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
            'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
            'ceres_loss_function': 'None',  # 文字列'None'に変更

            # ROS基本設定
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'scan_topic': '/scan',
            'use_map_saver': True,
            'mode': 'mapping',

            # デバッグ/パフォーマンス設定
            'debug_logging': False,
            'throttle_scans': 1,
            'transform_publish_period': 0.02,
            'map_update_interval': 1.0,
            'resolution': 0.025,
            'min_laser_range': 0.0,
            'max_laser_range': 8.0,
            'minimum_time_interval': 0.5,
            'transform_timeout': 1.0,
            'tf_buffer_duration': 30.0,
            'stack_size_to_use': 40000000,
            'enable_interactive_mode': True,

            # 一般動作パラメータ
            'use_scan_matching': True,
            'use_scan_barycenter': True,
            'minimum_travel_distance': 0.05,
            'minimum_travel_heading': 0.087,
            'scan_buffer_size': 10,
            'scan_buffer_maximum_scan_distance': 8.0,
            'link_match_minimum_response_fine': 0.1,
            'link_scan_maximum_distance': 1.5,
            'loop_search_maximum_distance': 3.0,
            'do_loop_closing': True,
            'loop_match_minimum_chain_size': 10,
            'loop_match_maximum_variance_coarse': 3.0,
            'loop_match_minimum_response_coarse': 0.35,
            'loop_match_minimum_response_fine': 0.45,

            # 相関計算パラメータ
            'correlation_search_space_dimension': 0.5,
            'correlation_search_space_resolution': 0.01,
            'correlation_search_space_smear_deviation': 0.1,

            # ループ検出パラメータ
            'loop_search_space_dimension': 8.0,
            'loop_search_space_resolution': 0.05,
            'loop_search_space_smear_deviation': 0.03,

            # スキャンマッチングパラメータ
            'distance_variance_penalty': 0.5,
            'angle_variance_penalty': 1.0,
            'fine_search_angle_offset': 0.00349,
            'coarse_search_angle_offset': 0.349,
            'coarse_angle_resolution': 0.0349,
            'minimum_angle_penalty': 0.9,
            'minimum_distance_penalty': 0.5,
            'use_response_expansion': True
        }
    }
}


def generate_launch_description():


    return LaunchDescription([
        Node(
            package="slam_toolbox",
            executable='sync_slam_toolbox_node',
            name='erk_slam_toolbox',
            output='screen',
            parameters=[slam_params],
            remappings = [
                ('/scan', f'/{NAMESPACE}/lidar/scan'),
            ],
        )
    ])