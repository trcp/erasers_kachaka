#!/bin/bash
source /home/roboworks/colcon_ws/install/setup.bash

echo $KACHAKA_IP
echo $KACHAKA_NAME

export API_GRPC_BRIDGE_SERVER_URI="${KACHAKA_IP}:${GRPC_PORT}"

ros2 launch erasers_kachaka_bringup bringup.launch.py
