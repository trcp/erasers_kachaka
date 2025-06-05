#!/bin/bash

echo "Map Path: ${MAP_PATTH}"

export API_GRPC_BRIDGE_SERVER_URI="${KACHAKA_IP}:${GRPC_PORT}"

. /home/roboworks/colcon_ws/install/setup.bash

ros2 launch erasers_kachaka_navigation navigation_launch.py map:=${MAP_PATH}
