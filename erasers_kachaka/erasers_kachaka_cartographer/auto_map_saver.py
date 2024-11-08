#!/usr/bin/env python3
from rclpy.node import Node
import rclpy

import subprocess
import time

def main():
    rclpy.init()
    node = Node("auto_map_saver")

    namespace = node.get_namespace()

    node.declare_parameter('map_name', 'map')
    node.declare_parameter('map_save_path', '/home/roboworks/education_ws/src/erasers_kachaka/erasers_kachaka/erasers_kachaka_cartographer/map')
    node.declare_parameter('save_late', 5)

    path = node.get_parameter('map_save_path').get_parameter_value().string_value
    name = node.get_parameter('map_name').get_parameter_value().string_value
    late = node.get_parameter('save_late').get_parameter_value().integer_value

    cmd1 = 'ros2 run nav2_map_server map_saver_cli -f \"%s/%s\" --ros-args -p map_subscribe_transient_local:=true -r __ns:=%s'%(path, name, namespace)

    while rclpy.ok():
        try:
            try:
                subprocess.run(cmd1,
                                shell=True,
                                check=True,
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE)
                node.get_logger().info('saved %s'%path)
            except subprocess.CalledProcessError as e:
                node.get_logger().error('map is not exists !')
            time.sleep(late)
            
        except KeyboardInterrupt:
            subprocess.run(cmd1,
                            shell=True,
                            stdout=subprocess.PIPE)
            node.get_logger().info('saved %s'%path)
            node.get_logger().info('Done ...')
            break

if __name__ == "__main__":
    main()