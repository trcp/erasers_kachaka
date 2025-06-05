#!/bin/bash

echo "Bringup Type: ${TYPE_BRINGUP}"
echo "Shelf Type: ${TYPE_SHELF}"

export API_GRPC_BRIDGE_SERVER_URI="${KACHAKA_IP}:${GRPC_PORT}"

. /home/roboworks/colcon_ws/install/setup.bash

ros2 launch erasers_kachaka_bringup bringup.launch.py bringup_type:=${TYPE_BRINGUP} shelf_type:=${TYPE_SHELF}
